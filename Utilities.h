#include <stddef.h>
#include "LoRa.h"
#include "ROM.h"
#include "Config.h"
#include "Framing.h"

void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
void led_tx_off() { digitalWrite(pin_led_tx, LOW); }

#ifdef USE_OLED
void draw_info(String text, uint8_t line, bool draw_now){
  uint8_t line_high = 9;
  display.fillRect(2, (line*line_high), display.width()-2, line_high, SSD1306_BLACK); // clear Line
  display.setCursor(2, line*line_high); // oled display
  display.setTextSize(0);
  display.setTextColor(SSD1306_WHITE);
  display.println(text);
  if (draw_now) {
    display.display(); // drawing commands to make them visible on screen!
  }
}
#endif

uint8_t getPrefByte(int16_t addr, uint8_t default_val) {
  //return prefbuf[addr];
  char cstr[16];
  sprintf(cstr, "%05d", addr);
  uint8_t data = preferences.getUChar(cstr, default_val);
  return data;
}

void putPrefByte(int16_t adr, uint8_t val) {
  char cstr[16];
  sprintf(cstr, "%05d", adr);
  size_t ok = preferences.putUChar(cstr, val);
}

boolean checkPref(int16_t adr, uint8_t val) {
  uint8_t data = getPrefByte(adr, 0);
  if (data != val) {
    putPrefByte(adr, val);
  }
  return data == val;
}

void led_indicate_error(int cycles) {
	bool forever = (cycles == 0) ? true : false;
	cycles = forever ? 1 : cycles;
	while(cycles > 0) {
        digitalWrite(pin_led_rx, HIGH);
        digitalWrite(pin_led_tx, LOW);
        delay(100);
        digitalWrite(pin_led_rx, LOW);
        digitalWrite(pin_led_tx, HIGH);
        delay(100);
        if (!forever) cycles--;
    }
    digitalWrite(pin_led_rx, LOW);
    digitalWrite(pin_led_tx, LOW);
}

void led_indicate_warning(int cycles) {
	bool forever = (cycles == 0) ? true : false;
	cycles = forever ? 1 : cycles;
	digitalWrite(pin_led_tx, HIGH);
	while(cycles > 0) {
        digitalWrite(pin_led_tx, LOW);
        delay(100);
        digitalWrite(pin_led_tx, HIGH);
        delay(100);
        if (!forever) cycles--;
    }
    digitalWrite(pin_led_tx, LOW);
}

void led_indicate_info(int cycles) {
	bool forever = (cycles == 0) ? true : false;
	cycles = forever ? 1 : cycles;
	while(cycles > 0) {
        digitalWrite(pin_led_rx, LOW);
        delay(100);
        digitalWrite(pin_led_rx, HIGH);
        delay(100);
        if (!forever) cycles--;
    }
    digitalWrite(pin_led_rx, LOW);
}

uint8_t led_standby_min = 1;
uint8_t led_standby_max = 40;
uint8_t led_standby_value = led_standby_min;
int8_t  led_standby_direction = 0;
uint32_t led_standby_ticks = 0;
uint32_t led_standby_wait = 11000;
void led_indicate_standby() {
	led_standby_ticks++;
	if (led_standby_ticks > led_standby_wait) {
		led_standby_ticks = 0;
		if (led_standby_value <= led_standby_min) {
			led_standby_direction = 1;
		} else if (led_standby_value >= led_standby_max) {
			led_standby_direction = -1;
		}
		led_standby_value += led_standby_direction;
		digitalWrite(pin_led_tx, 0);
	}
}

void led_indicate_not_ready() {
	led_standby_ticks++;
	if (led_standby_ticks > led_standby_wait) {
		led_standby_ticks = 0;
		if (led_standby_value <= led_standby_min) {
			led_standby_direction = 1;
		} else if (led_standby_value >= led_standby_max) {
			led_standby_direction = -1;
		}
		led_standby_value += led_standby_direction;
		digitalWrite(pin_led_rx, 0);
	}
}

void escapedSerialWrite(uint8_t byte) {
	if (byte == FEND) { port->write(FESC); byte = TFEND; }
  if (byte == FESC) { port->write(FESC); byte = TFESC; }
  port->write(byte);
}

void kiss_indicate_error(uint8_t error_code) {
	port->write(FEND);
	port->write(CMD_ERROR);
	port->write(error_code);
	port->write(FEND);
}

void kiss_indicate_radiostate() {
	port->write(FEND);
	port->write(CMD_RADIO_STATE);
	port->write(radio_online);
	port->write(FEND);
}

void kiss_indicate_stat_rx() {
	port->write(FEND);
	port->write(CMD_STAT_RX);
	escapedSerialWrite(stat_rx>>24);
	escapedSerialWrite(stat_rx>>16);
	escapedSerialWrite(stat_rx>>8);
	escapedSerialWrite(stat_rx);
	port->write(FEND);
}

void kiss_indicate_stat_tx() {
	port->write(FEND);
	port->write(CMD_STAT_TX);
	escapedSerialWrite(stat_tx>>24);
	escapedSerialWrite(stat_tx>>16);
	escapedSerialWrite(stat_tx>>8);
	escapedSerialWrite(stat_tx);
	port->write(FEND);
}

void kiss_indicate_stat_rssi() {
	uint8_t packet_rssi_val = (uint8_t)(last_rssi+rssi_offset);
	port->write(FEND);
	port->write(CMD_STAT_RSSI);
	escapedSerialWrite(packet_rssi_val);
	port->write(FEND);
}

void kiss_indicate_stat_snr() {
	port->write(FEND);
	port->write(CMD_STAT_SNR);
	escapedSerialWrite(last_snr_raw);
	port->write(FEND);
}

void kiss_indicate_radio_lock() {
	port->write(FEND);
	port->write(CMD_RADIO_LOCK);
	port->write(radio_locked);
	port->write(FEND);
}

void kiss_indicate_spreadingfactor() {
	port->write(FEND);
	port->write(CMD_SF);
	port->write((uint8_t)lora_sf);
	port->write(FEND);
}

void kiss_indicate_codingrate() {
	port->write(FEND);
	port->write(CMD_CR);
	port->write((uint8_t)lora_cr);
	port->write(FEND);
}

void kiss_indicate_implicit_length() {
	port->write(FEND);
	port->write(CMD_IMPLICIT);
	port->write(implicit_l);
	port->write(FEND);
}

void kiss_indicate_txpower() {
	port->write(FEND);
	port->write(CMD_TXPOWER);
	port->write((uint8_t)lora_txp);
	port->write(FEND);
}

void kiss_indicate_bandwidth() {
	port->write(FEND);
	port->write(CMD_BANDWIDTH);
	escapedSerialWrite(lora_bw>>24);
	escapedSerialWrite(lora_bw>>16);
	escapedSerialWrite(lora_bw>>8);
	escapedSerialWrite(lora_bw);
	port->write(FEND);
}

void kiss_indicate_frequency() {
	port->write(FEND);
	port->write(CMD_FREQUENCY);
	escapedSerialWrite(lora_freq>>24);
	escapedSerialWrite(lora_freq>>16);
	escapedSerialWrite(lora_freq>>8);
	escapedSerialWrite(lora_freq);
	port->write(FEND);
}

void kiss_indicate_random(uint8_t byte) {
	port->write(FEND);
	port->write(CMD_RANDOM);
	port->write(byte);
	port->write(FEND);
}

void kiss_indicate_ready() {
	port->write(FEND);
	port->write(CMD_READY);
	port->write(0x01);
	port->write(FEND);
}

void kiss_indicate_not_ready() {
	port->write(FEND);
	port->write(CMD_READY);
	port->write(0x00);
	port->write(FEND);
}

void kiss_indicate_promisc() {
	port->write(FEND);
	port->write(CMD_PROMISC);
	if (promisc) {
		port->write(0x01);
	} else {
		port->write(0x00);
	}
	port->write(FEND);
}

void kiss_indicate_detect() {
	port->write(FEND);
	port->write(CMD_DETECT);
	port->write(DETECT_RESP);
	port->write(FEND);
}

void kiss_indicate_version() {
	port->write(FEND);
	port->write(CMD_FW_VERSION);
	port->write(MAJ_VERS);
	port->write(MIN_VERS);
	port->write(FEND);
}

bool isSplitPacket(uint8_t header) {
	return (header & FLAG_SPLIT);
}

uint8_t packetSequence(uint8_t header) {
	return header >> 4;
}

void getPacketData(int len) {
	while (len--) {
		pbuf[read_len++] = LoRa.read();
	}
}

void setSpreadingFactor() {
	if (radio_online) LoRa.setSpreadingFactor(lora_sf);
  #ifdef USE_OLED
  draw_info("sf: "+String(lora_sf),4,true);
  #endif
}

void setCodingRate() {
	if (radio_online) LoRa.setCodingRate4(lora_cr);
  #ifdef USE_OLED
  draw_info("cr: "+String(lora_cr),5,true);
  #endif
}

void set_implicit_length(uint8_t len) {
	implicit_l = len;
	if (implicit_l != 0) {
		implicit = true;
	} else {
		implicit = false;
	}
}

void setTXPower() {
  if (radio_online) {
    if (model == MODEL_A4) LoRa.setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);
    if (model == MODEL_A9) LoRa.setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_B1) LoRa.setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
#ifdef USE_OLED
    draw_info("txp: " + String(lora_txp), 3, true);
#endif
  }
}


void getBandwidth() {
	if (radio_online) {
			lora_bw = LoRa.getSignalBandwidth();
	}
}

void setBandwidth() {
	if (radio_online) {
		LoRa.setSignalBandwidth(lora_bw);
		getBandwidth();
   #ifdef USE_OLED
    draw_info("bw: "+String(lora_bw),1,true);
    #endif
	}
}

void getFrequency() {
	if (radio_online) {
		lora_freq = LoRa.getFrequency();
	}
}

void setFrequency() {
	if (radio_online) {
		LoRa.setFrequency(lora_freq);
		getFrequency();
   #ifdef USE_OLED
    draw_info("freq: "+String(lora_freq),2,true);
    #endif
	}
}

uint8_t getRandom() {
	if (radio_online) {
		return LoRa.random();
	} else {
		return 0x00;
	}
}

void promisc_enable() {
	promisc = true;
}

void promisc_disable() {
	promisc = false;
}

void eeprom_dump_info() {
  for (int16_t addr = ADDR_PRODUCT; addr <= ADDR_INFO_LOCK; addr++) {
    uint8_t byte = getPrefByte(addr, 0);
    escapedSerialWrite(byte);
  }
}

void eeprom_dump_config() {
  for (int16_t addr = ADDR_CONF_SF; addr <= ADDR_CONF_OK; addr++) {
    uint8_t byte = getPrefByte(addr, 0);
    escapedSerialWrite(byte);
  }
}

void eeprom_dump_all() {
  for (int16_t addr = 0; addr < EEPROM_RESERVED; addr++) {
    uint8_t byte = getPrefByte(addr, 0);
    escapedSerialWrite(byte);
  }
}

void kiss_dump_eeprom() {
	port->write(FEND);
	port->write(CMD_ROM_READ);
	eeprom_dump_all();
	port->write(FEND);
}

void eeprom_write(uint8_t addr, uint8_t wbyte) {
  if (addr >= 0 && addr < EEPROM_RESERVED) {
    putPrefByte((int16_t)addr, wbyte);
  } else {
    kiss_indicate_error(ERROR_EEPROM_LOCKED);
  }
}

void eeprom_erase() {
  preferences.clear();
  while (true) {
    led_tx_on();
    led_rx_off();
  }
}

bool eeprom_lock_set() {
  uint8_t INFO_LOCK = getPrefByte(ADDR_INFO_LOCK, 0);
  if (INFO_LOCK == INFO_LOCK_BYTE) {
    return true;
  } else {
    return false;
  }
}

bool eeprom_product_valid() {
  return true;
}

bool eeprom_model_valid() {
  model = getPrefByte(ADDR_MODEL, 0);
  if (model == MODEL_B1) {
    return true;
  } else {
    return false;
  }
}

bool eeprom_hwrev_valid() {
  hwrev = getPrefByte(ADDR_HW_REV, 0);
  if (hwrev != 0x00 && hwrev != 0xFF) {
    return true;
  } else {
    return false;
  }
}

bool eeprom_have_conf() {
  uint8_t CONF_OK = getPrefByte(ADDR_CONF_OK, 0x00);
  if (CONF_OK == CONF_OK_BYTE) {
    return true;
  } else {
    return false;
  }
}

void eeprom_conf_load() {
  if (eeprom_have_conf()) {
    lora_sf = getPrefByte(ADDR_CONF_SF, 0);
    lora_cr = getPrefByte(ADDR_CONF_CR, 0);
    lora_txp = getPrefByte(ADDR_CONF_TXP, 0);
    lora_freq = (uint32_t)getPrefByte(ADDR_CONF_FREQ, 0) << 24 | (uint32_t)getPrefByte(ADDR_CONF_FREQ + 0x01, 0) << 16 | (uint32_t)getPrefByte(ADDR_CONF_FREQ + 0x02, 0) << 8 | (uint32_t)getPrefByte(ADDR_CONF_FREQ + 0x03, 0);
    lora_bw = (uint32_t)getPrefByte(ADDR_CONF_BW, 0) << 24 | (uint32_t)getPrefByte(ADDR_CONF_BW + 0x01, 0) << 16 | (uint32_t)getPrefByte(ADDR_CONF_BW + 0x02, 0) << 8 | (uint32_t)getPrefByte(ADDR_CONF_BW + 0x03, 0);
  }
}

void eeprom_conf_save() {
  if (hw_ready && radio_online) {
    putPrefByte(ADDR_CONF_SF, lora_sf);
    putPrefByte(ADDR_CONF_CR, lora_cr);
    putPrefByte(ADDR_CONF_TXP, lora_txp);

    putPrefByte(ADDR_CONF_BW, lora_bw >> 24);
    putPrefByte(ADDR_CONF_BW + 0x01, lora_bw >> 16);
    putPrefByte(ADDR_CONF_BW + 0x02, lora_bw >> 8);
    putPrefByte(ADDR_CONF_BW + 0x03, lora_bw);

    putPrefByte(ADDR_CONF_FREQ + 0x00, lora_freq >> 24);
    putPrefByte(ADDR_CONF_FREQ + 0x01, lora_freq >> 16);
    putPrefByte(ADDR_CONF_FREQ + 0x02, lora_freq >> 8);
    putPrefByte(ADDR_CONF_FREQ + 0x03, lora_freq);

    putPrefByte(ADDR_CONF_OK, CONF_OK_BYTE);
    led_indicate_info(10);
  } else {
    led_indicate_warning(10);
  }
}

void eeprom_conf_delete() {
  putPrefByte(ADDR_CONF_OK, 0x00);
}

void unlock_rom() {
  led_indicate_error(50);
  eeprom_erase();
}

typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
} FIFOBuffer;

inline bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

inline bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline unsigned char fifo_pop(FIFOBuffer *f) {
  if (f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}

static inline bool fifo_isempty_locked(const FIFOBuffer *f) {
  bool result;
  portENTER_CRITICAL_ISR(&timerMux);
  result = fifo_isempty(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return result;
}

static inline bool fifo_isfull_locked(const FIFOBuffer *f) {
  bool result;
  portENTER_CRITICAL_ISR(&timerMux);
  result = fifo_isfull(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return result;
}

static inline void fifo_push_locked(FIFOBuffer *f, unsigned char c) {
  portENTER_CRITICAL_ISR(&timerMux);
  fifo_push(f, c);
  portEXIT_CRITICAL_ISR(&timerMux);
}

static inline unsigned char fifo_pop_locked(FIFOBuffer *f) {
  unsigned char c;
  portENTER_CRITICAL_ISR(&timerMux);
  c = fifo_pop(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return c;
}

inline void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

typedef struct FIFOBuffer16
{
  size_t *begin;
  size_t *end;
  size_t * volatile head;
  size_t * volatile tail;
} FIFOBuffer16;

inline bool fifo16_isempty(const FIFOBuffer16 *f) {
  return f->head == f->tail;
}

inline bool fifo16_isfull(const FIFOBuffer16 *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo16_push(FIFOBuffer16 *f, size_t c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline size_t fifo16_pop(FIFOBuffer16 *f) {
  if (f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo16_flush(FIFOBuffer16 *f) {
  f->head = f->tail;
}

static inline bool fifo16_isempty_locked(const FIFOBuffer16 *f) {
  bool result;
  portENTER_CRITICAL_ISR(&timerMux);
  result = fifo16_isempty(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return result;
}

static inline bool fifo16_isfull_locked(const FIFOBuffer16 *f) {
  bool result;
  portENTER_CRITICAL_ISR(&timerMux);
  result = fifo16_isfull(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return result;
}

static inline void fifo16_push_locked(FIFOBuffer16 *f, size_t c) {
  portENTER_CRITICAL_ISR(&timerMux);
  fifo16_push(f, c);
  portEXIT_CRITICAL_ISR(&timerMux);
}

static inline size_t fifo16_pop_locked(FIFOBuffer16 *f) {
  size_t c;
  portENTER_CRITICAL_ISR(&timerMux);
  c = fifo16_pop(f);
  portEXIT_CRITICAL_ISR(&timerMux);
  return c;
}

inline void fifo16_init(FIFOBuffer16 *f, size_t *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline size_t fifo16_len(FIFOBuffer16 *f) {
  return (f->end - f->begin);
}
