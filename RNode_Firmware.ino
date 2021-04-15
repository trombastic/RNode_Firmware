#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#ifdef ESP32
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  hw_timer_t * timer = NULL;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  #include <Preferences.h>
  Preferences preferences;
  #define OLED_RESET   16 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define OLED_SDA     4 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define OLED_SCL     15 // Reset pin # (or -1 if sharing Arduino reset pin)
  
  #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  TwoWire twi = TwoWire(1); // create our own TwoWire instance
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &twi, OLED_RESET);

#endif
#include "Utilities.h"
FIFOBuffer serialFIFO;
uint8_t serialBuffer[CONFIG_UART_BUFFER_SIZE+1];

FIFOBuffer16 packet_starts;
size_t packet_starts_buf[CONFIG_QUEUE_MAX_LENGTH+1];

FIFOBuffer16 packet_lengths;
size_t packet_lengths_buf[CONFIG_QUEUE_MAX_LENGTH+1];

uint8_t packet_queue[CONFIG_QUEUE_SIZE];

volatile uint8_t queue_height = 0;
volatile size_t queued_bytes = 0;
volatile size_t queue_cursor = 0;
volatile size_t current_packet_start = 0;
volatile bool serial_buffering = false;
#ifdef ESP32
volatile int16_t bytes_received = 0;
volatile int16_t packets_received = 0;
#endif
char sbuf[128];


void setup() {
  // Seed the PRNG
  randomSeed(analogRead(0));

  // Initialise serial communication
  memset(serialBuffer, 0, sizeof(serialBuffer));
  fifo_init(&serialFIFO, serialBuffer, CONFIG_UART_BUFFER_SIZE);

  Serial.begin(serial_baudrate);
  while (!Serial);
  #ifdef ESP32
    preferences.begin("RNode", false);
    //preferences.clear();
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
    twi.begin(OLED_SDA,OLED_SCL);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    // Clear the buffer
    display.clearDisplay();
    draw_info("promisc: "+String(promisc),0,false);
    draw_info("bw: "+String(lora_bw),1,false);
    draw_info("freq: "+String(lora_freq),2,false);
    draw_info("txp: "+String(lora_txp),3,false);
    draw_info("sf: "+String(lora_sf),4,false);
    draw_info("cr: "+String(lora_cr),5,true);
    draw_info("status: offline",6,true);
    //load_defaults();
  #else
    while (!Serial);
  #endif
  // Configure input and output pins
  pinMode(pin_led_rx, OUTPUT);
  pinMode(pin_led_tx, OUTPUT);

  // Initialise buffers
  memset(pbuf, 0, sizeof(pbuf));
  memset(cbuf, 0, sizeof(cbuf));
  
  memset(packet_queue, 0, sizeof(packet_queue));

  memset(packet_starts_buf, 0, sizeof(packet_starts_buf));
  fifo16_init(&packet_starts, packet_starts_buf, CONFIG_QUEUE_MAX_LENGTH);
  
  memset(packet_lengths_buf, 0, sizeof(packet_starts_buf));
  fifo16_init(&packet_lengths, packet_lengths_buf, CONFIG_QUEUE_MAX_LENGTH);

  // Set chip select, reset and interrupt
  // pins for the LoRa module
  LoRa.setPins(pin_cs, pin_reset, pin_dio);

  led_indicate_info(2);
  delay(1000);

  serial_interrupt_init();

  // Validate board health, EEPROM and config
  validateStatus();
}

void lora_receive() {
  if (!implicit) {
    LoRa.receive();
  } else {
    LoRa.receive(implicit_l);
  }
}

bool startRadio() {
  update_radio_lock();
  if (!radio_online) {
    if (!radio_locked && hw_ready) {
      if (!LoRa.begin(lora_freq)) {
        // The radio could not be started.
        // Indicate this failure over both the
        // serial port and with the onboard LEDs
        kiss_indicate_error(ERROR_INITRADIO);
        led_indicate_error(0);
      } else {
        radio_online = true;
        #ifdef ESP32
        draw_info("freq: "+String(lora_freq),2,false);
        #endif
        setTXPower();
        setBandwidth();
        setSpreadingFactor();
        setCodingRate();
        getFrequency();

        LoRa.enableCrc();
        LoRa.onReceive(receiveCallback);

        lora_receive();

        // Flash an info pattern to indicate
        // that the radio is now on
        led_indicate_info(3);
        #ifdef ESP32
        draw_info("promisc: "+String(promisc),0,false);
        draw_info("status: online",6,true);
        #endif
      }

    } else {
      // Flash a warning pattern to indicate
      // that the radio was locked, and thus
      // not started
      led_indicate_warning(3);
      #ifdef ESP32
      draw_info("status: offline",6,true);
      #endif
    }
  } else {
    // If radio is already on, we silently
    // ignore the request.
  }
}

void stopRadio() {
  LoRa.end();
  radio_online = false;
  #ifdef ESP32
  draw_info("status: offline",6,true);
  #endif
}

void update_radio_lock() {
  if (lora_freq != 0 && lora_bw != 0 && lora_txp != 0xFF && lora_sf != 0) {
    radio_locked = false;
  } else {
    radio_locked = true;
  }
}

void receiveCallback(int16_t packet_size) {
  bytes_received = packet_size;
  if (!promisc) {
    // The standard operating mode allows large
    // packets with a payload up to 500 bytes,
    // by combining two raw LoRa packets.
    // We read the 1-byte header and extract
    // packet sequence number and split flags
    uint8_t header   = LoRa.read(); packet_size--;
    uint8_t sequence = packetSequence(header);
    bool    ready    = false;
    
    
    if (isSplitPacket(header) && seq == SEQ_UNSET) {
      // This is the first part of a split
      // packet, so we set the seq variable
      // and add the data to the buffer
      read_len = 0;
      seq = sequence;
      last_rssi = LoRa.packetRssi();
      last_snr_raw = LoRa.packetSnrRaw();
      getPacketData(packet_size); // updates read_len
    } else if (isSplitPacket(header) && seq == sequence) {
      // This is the second part of a split
      // packet, so we add it to the buffer
      // and set the ready flag.
      last_rssi = (last_rssi+LoRa.packetRssi())/2;
      last_snr_raw = (last_snr_raw+LoRa.packetSnrRaw())/2;
      getPacketData(packet_size); // updates read_len
      seq = SEQ_UNSET;
      ready = true;
    } else if (isSplitPacket(header) && seq != sequence) {
      // This split packet does not carry the
      // same sequence id, so we must assume
      // that we are seeing the first part of
      // a new split packet.
      read_len = 0;
      seq = sequence;
      last_rssi = LoRa.packetRssi();
      last_snr_raw = LoRa.packetSnrRaw();
      getPacketData(packet_size); // updates read_len
    } else if (!isSplitPacket(header)) {
      // This is not a split packet, so we
      // just read it and set the ready
      // flag to true.

      if (seq != SEQ_UNSET) {
        // If we already had part of a split
        // packet in the buffer, we clear it.
        read_len = 0;
        seq = SEQ_UNSET;
      }

      last_rssi = LoRa.packetRssi();
      last_snr_raw = LoRa.packetSnrRaw();
      getPacketData(packet_size); // updates read_len
      ready = true;
    }

    if (ready) {
      // We first signal the RSSI of the
      // recieved packet to the host.
      kiss_indicate_stat_rssi();
      kiss_indicate_stat_snr();

      // And then write the entire packet
      Serial.write(FEND);
      Serial.write(CMD_DATA);
      for (int16_t i = 0; i < read_len; i++) {
        uint8_t byte = pbuf[i];
        if (byte == FEND) { Serial.write(FESC); byte = TFEND; }
        if (byte == FESC) { Serial.write(FESC); byte = TFESC; }
        Serial.write(byte);
      }
      Serial.write(FEND);
      read_len = 0;
    }
  } else {
    // In promiscuous mode, raw packets are
    // output directly over to the host
    read_len = 0;
    last_rssi = LoRa.packetRssi();
    last_snr_raw = LoRa.packetSnrRaw();
    getPacketData(packet_size); // updates read_len

    // We first signal the RSSI of the
    // recieved packet to the host.
    kiss_indicate_stat_rssi();
    kiss_indicate_stat_snr();

    // And then write the entire packet
    Serial.write(FEND);
    Serial.write(CMD_DATA);
    for (int16_t i = 0; i < read_len; i++) {
      uint8_t byte = pbuf[i];
      if (byte == FEND) { Serial.write(FESC); byte = TFEND; }
      if (byte == FESC) { Serial.write(FESC); byte = TFESC; }
      Serial.write(byte);
    }
    Serial.write(FEND);
    read_len = 0;
  }
}

bool queueFull() {
  return (queue_height >= CONFIG_QUEUE_MAX_LENGTH || queued_bytes >= CONFIG_QUEUE_SIZE);
}

volatile bool queue_flushing = false;
void flushQueue(void) {
  if (!queue_flushing) {
    queue_flushing = true;

    size_t processed = 0;
    while (!fifo16_isempty_locked(&packet_starts)) {
      size_t start = fifo16_pop(&packet_starts);
      size_t length = fifo16_pop(&packet_lengths);

      if (length >= MIN_L && length <= MTU) {
        for (size_t i = 0; i < length; i++) {
          size_t pos = (start+i)%CONFIG_QUEUE_SIZE;
          tbuf[i] = packet_queue[pos];
        }

        transmit(length);
        processed++;
      }
    }
  }

  queue_height = 0;
  queued_bytes = 0;
  queue_flushing = false;
}

void transmit(size_t size) {
  if (radio_online) {
    if (!promisc) {
      led_tx_on();
      size_t  written = 0;
      size_t  written_wrap = 1;
      uint8_t header  = random(256) & 0xF0;

      if (size > SINGLE_MTU - HEADER_L) {
        header = header | FLAG_SPLIT;
      }

      LoRa.beginPacket();
      LoRa.write(header); written++;
      
      for (size_t i; i < size; i++) {
        LoRa.write(tbuf[i]);  

        written++;

        if (written == 255) {
          LoRa.endPacket();
          LoRa.beginPacket();
          LoRa.write(header);
          written = 1;
          written_wrap++;
        }
      }

      LoRa.endPacket();
      led_tx_off();
      #ifdef ESP32
      draw_info("transmit: "+String(written*written_wrap),6,true);
      #endif
      lora_receive();
    } else {
      // In promiscuous mode, we only send out
      // plain raw LoRa packets with a maximum
      // payload of 255 bytes
      led_tx_on();
      size_t  written = 0;
      // Cap packets at 255 bytes
      if (size > SINGLE_MTU) {
        size = SINGLE_MTU;
      }

      // If implicit header mode has been set,
      // set packet length to payload data length
      if (!implicit) {
        LoRa.beginPacket();
      } else {
        LoRa.beginPacket(size);
      }

      for (size_t i; i < size; i++) {
        LoRa.write(tbuf[i]);

        written++;
      }
      LoRa.endPacket();
      led_tx_off();
      #ifdef ESP32
      draw_info("transmit: "+String(written),6,true);
      #endif
      lora_receive();
    }
  } else {
    kiss_indicate_error(ERROR_TXFAILED);
    led_indicate_error(5);
  }
}

void serialCallback(uint8_t sbyte) {
  if (IN_FRAME && sbyte == FEND && command == CMD_DATA) {
    IN_FRAME = false;

    if (!fifo16_isfull(&packet_starts) && queued_bytes < CONFIG_QUEUE_SIZE) {
        size_t s = current_packet_start;
        size_t e = queue_cursor-1; if (e == -1) e = CONFIG_QUEUE_SIZE-1;
        size_t l;

        if (s != e) {
            l = (s < e) ? e - s + 1 : CONFIG_QUEUE_SIZE - s + e + 1;
        } else {
            l = 1;
        }

        if (l >= MIN_L) {
            queue_height++;

            fifo16_push(&packet_starts, s);
            fifo16_push(&packet_lengths, l);

            current_packet_start = queue_cursor;
        }

    }

  } else if (sbyte == FEND) {
    IN_FRAME = true;
    command = CMD_UNKNOWN;
    frame_len = 0;
  } else if (IN_FRAME && frame_len < MTU) {
    // Have a look at the command byte first
    if (frame_len == 0 && command == CMD_UNKNOWN) {
        command = sbyte;
    } else if (command == CMD_DATA) {
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (queue_height < CONFIG_QUEUE_MAX_LENGTH && queued_bytes < CONFIG_QUEUE_SIZE) {
              queued_bytes++;
              packet_queue[queue_cursor++] = sbyte;
              if (queue_cursor == CONFIG_QUEUE_SIZE) queue_cursor = 0;
            }
        }
    } else if (command == CMD_FREQUENCY) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            cbuf[frame_len++] = sbyte;
        }

        if (frame_len == 4) {
          uint32_t freq = (uint32_t)cbuf[0] << 24 | (uint32_t)cbuf[1] << 16 | (uint32_t)cbuf[2] << 8 | (uint32_t)cbuf[3];

          if (freq == 0) {
            kiss_indicate_frequency();
          } else {
            lora_freq = freq;
            if (op_mode == MODE_HOST) setFrequency();
            kiss_indicate_frequency();
          }
        }
    } else if (command == CMD_BANDWIDTH) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            cbuf[frame_len++] = sbyte;
        }

        if (frame_len == 4) {
          uint32_t bw = (uint32_t)cbuf[0] << 24 | (uint32_t)cbuf[1] << 16 | (uint32_t)cbuf[2] << 8 | (uint32_t)cbuf[3];

          if (bw == 0) {
            kiss_indicate_bandwidth();
          } else {
            lora_bw = bw;
            if (op_mode == MODE_HOST) setBandwidth();
            kiss_indicate_bandwidth();
          }
        }
    } else if (command == CMD_TXPOWER) {
      if (sbyte == 0xFF) {
        kiss_indicate_txpower();
      } else {
        int16_t txp = sbyte;
        if (txp > 17) txp = 17;

        lora_txp = txp;
        if (op_mode == MODE_HOST) setTXPower();
        kiss_indicate_txpower();
      }
    } else if (command == CMD_SF) {
      if (sbyte == 0xFF) {
        kiss_indicate_spreadingfactor();
      } else {
        int16_t sf = sbyte;
        if (sf < 6) sf = 6;
        if (sf > 12) sf = 12;

        lora_sf = sf;
        if (op_mode == MODE_HOST) setSpreadingFactor();
        kiss_indicate_spreadingfactor();
      }
    } else if (command == CMD_CR) {
      if (sbyte == 0xFF) {
        kiss_indicate_codingrate();
      } else {
        int16_t cr = sbyte;
        if (cr < 5) cr = 5;
        if (cr > 8) cr = 8;

        lora_cr = cr;
        if (op_mode == MODE_HOST) setCodingRate();
        kiss_indicate_codingrate();
      }
    } else if (command == CMD_IMPLICIT) {
      set_implicit_length(sbyte);
      kiss_indicate_implicit_length();
    } else if (command == CMD_RADIO_STATE) {
      if (sbyte == 0xFF) {
        kiss_indicate_radiostate();
      } else if (sbyte == 0x00) {
        stopRadio();
        kiss_indicate_radiostate();
      } else if (sbyte == 0x01) {
        startRadio();
        kiss_indicate_radiostate();
      }
    } else if (command == CMD_STAT_RX) {
      kiss_indicate_stat_rx();
    } else if (command == CMD_STAT_TX) {
      kiss_indicate_stat_tx();
    } else if (command == CMD_STAT_RSSI) {
      kiss_indicate_stat_rssi();
    } else if (command == CMD_RADIO_LOCK) {
      update_radio_lock();
      kiss_indicate_radio_lock();
    } else if (command == CMD_BLINK) {
      led_indicate_info(sbyte);
    } else if (command == CMD_RANDOM) {
      kiss_indicate_random(getRandom());
    } else if (command == CMD_DETECT) {
      if (sbyte == DETECT_REQ) {
        kiss_indicate_detect();
      }
    } else if (command == CMD_PROMISC) {
      if (sbyte == 0x01) {
        promisc_enable();
      } else if (sbyte == 0x00) {
        promisc_disable();
      }
      kiss_indicate_promisc();
      #ifdef ESP32
      draw_info("promisc: "+String(promisc),0,true);
      #endif
    } else if (command == CMD_READY) {
      if (!queueFull()) {
        kiss_indicate_ready();
      } else {
        kiss_indicate_not_ready();
      }
    } else if (command == CMD_UNLOCK_ROM) {
      if (sbyte == ROM_UNLOCK_BYTE) {
        unlock_rom();
      }
    } else if (command == CMD_ROM_READ) {
      kiss_dump_eeprom();
    } else if (command == CMD_ROM_WRITE) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            cbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          eeprom_write(cbuf[0], cbuf[1]);
        }
    } else if (command == CMD_FW_VERSION) {
      kiss_indicate_version();
    } else if (command == CMD_CONF_SAVE) {
      eeprom_conf_save();
    } else if (command == CMD_CONF_DELETE) {
      eeprom_conf_delete();
    }
  }
}

void updateModemStatus() {
  uint8_t status = LoRa.modemStatus();
  last_status_update = millis();
  if (status & SIG_DETECT == SIG_DETECT) { stat_signal_detected = true; } else { stat_signal_detected = false; }
  if (status & SIG_SYNCED == SIG_SYNCED) { stat_signal_synced = true; } else { stat_signal_synced = false; }
  if (status & RX_ONGOING == RX_ONGOING) { stat_rx_ongoing = true; } else { stat_rx_ongoing = false; }

  if (stat_signal_detected || stat_signal_synced || stat_rx_ongoing) {
    if (dcd_count < dcd_threshold) {
      dcd_count++;
      dcd = true;
    } else {
      dcd = true;
      dcd_led = true;
    }
  } else {
    if (dcd_count > 0) {
      dcd_count--;
    } else {
      dcd_led = false;
    }
    dcd = false;
  }

  if (dcd_led) {
    led_rx_on();
  } else {
    led_rx_off();
  }
}

void checkModemStatus() {
  if (millis()-last_status_update >= status_interval_ms) {
    updateModemStatus();
  }
}

void validateStatus() {
  if (eeprom_lock_set()) {
    if (eeprom_product_valid() && eeprom_model_valid() && eeprom_hwrev_valid()) {
      if (eeprom_checksum_valid()) {
        hw_ready = true;

        if (eeprom_have_conf()) {
          eeprom_conf_load();
          op_mode = MODE_TNC;
          startRadio();
        }
      }
    } else {
      hw_ready = false;
    }
  } else {
    hw_ready = false;
  }
}

void loop() {
  if (radio_online) {
    checkModemStatus();

    if (queue_height > 0) {
      if (!dcd_waiting) updateModemStatus();

      if (!dcd && !dcd_led) {
        if (dcd_waiting) delay(lora_rx_turnaround_ms);

        updateModemStatus();

        if (!dcd) {
          dcd_waiting = false;

          flushQueue();
          
        }
      } else {
        dcd_waiting = true;
      }
    }
  
  } else {
    if (hw_ready) {
      led_indicate_standby();
    } else {
      led_indicate_not_ready();
      stopRadio();
      validateStatus(); // delete me
    }
  }
  if (!fifo_isempty_locked(&serialFIFO)) serial_poll();
  #ifdef ESP32
  if (bytes_received != 0){
    packets_received++;
    draw_info("rssi: "+String(last_rssi),0,false);
    draw_info("received: "+String(bytes_received) + "(" + String(packets_received) + ")",5,false);
    char blestr[bytes_received];
    for (size_t i; i < bytes_received; i++) {
      char tmp = pbuf[i];
      if (tmp>=32 & tmp<=126){
        blestr[i] = tmp;
      }else{
        blestr[i] = 0;
        }
      }
    draw_info(blestr,6,true);
    bytes_received = 0;
  }
  
  uint8_t  c = 0;
  while (c < 20 && Serial.available()) {
    c++;
    if (!fifo_isfull_locked(&serialFIFO)) {
      fifo_push_locked(&serialFIFO, Serial.read());
    }
  }
  #endif
}

volatile bool serial_polling = false;
void serial_poll() {
  serial_polling = true;

  while (!fifo_isempty_locked(&serialFIFO)) {
    char sbyte = fifo_pop(&serialFIFO);
    serialCallback(sbyte);
  }

  serial_polling = false;
}


#ifdef ESP32
#define MAX_CYCLES 20
void IRAM_ATTR buffer_serial() {
  portENTER_CRITICAL_ISR(&timerMux);
#else
#define MAX_CYCLES 20
void buffer_serial() {
#endif
  if (!serial_buffering) {
    serial_buffering = true;

    uint8_t c = 0;
    while (c < MAX_CYCLES && Serial.available()) {
      c++;

      if (!fifo_isfull_locked(&serialFIFO)) {
        fifo_push_locked(&serialFIFO, Serial.read());
      }
    }

    serial_buffering = false;
  }
  #ifdef ESP32
  portEXIT_CRITICAL_ISR(&timerMux);
  #endif
}

#ifdef ESP32
void serial_interrupt_init() {
    //timer = timerBegin(0, 80, true); // CLK is 250 MHz
    //timerAttachInterrupt(timer, &buffer_serial, true);
    //timerAlarmWrite(timer, 1000, true);
    //timerAlarmEnable(timer);
}
#else
void serial_interrupt_init() {
  TCCR3A = 0;
  TCCR3B = _BV(CS10) |
           _BV(WGM33)|
           _BV(WGM32);

  // Buffer incoming frames every 1ms
  ICR3 = 16000;

  TIMSK3 = _BV(ICIE3);
}

ISR(TIMER3_CAPT_vect) {
  buffer_serial();
}
#endif
