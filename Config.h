#include "ROM.h"

#ifndef CONFIG_H
	#define CONFIG_H

	#define MAJ_VERS  0x01
	#define MIN_VERS  0x12

	#define MCU_328P  0x90
	#define MCU_1284P 0x91
  #define MCU_ESP32 0x92

	#define MODE_HOST 0x11
	#define MODE_TNC  0x12

	#if defined(__AVR_ATmega1284P__)
		#define MCU_VARIANT MCU_1284P
		#warning "Firmware is being compiled for atmega1284p based boards"
	#elif defined(ESP32)
    		#define MCU_VARIANT MCU_ESP32
    		#warning "Firmware is being compiled for esp32 based boards"
	#else
		#error "The firmware cannot be compiled for the selected MCU variant"
	#endif

	#define MTU   	   500
	#define SINGLE_MTU 255
	#define HEADER_L   1
	#define MIN_L	   1

	#define CMD_L      4

	// MCU dependent configuration parameters

	#if MCU_VARIANT == MCU_1284P
		const int16_t pin_cs = 4;
		const int16_t pin_reset = 3;
		const int16_t pin_dio = 2;
		const int16_t pin_led_rx = 12;
		const int16_t pin_led_tx = 13;

		// TODO: Reset
		#define CONFIG_UART_BUFFER_SIZE 6144
		#define CONFIG_QUEUE_SIZE 6144
		#define CONFIG_QUEUE_MAX_LENGTH 250

		#define EEPROM_SIZE 4096
		#define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
	#elif MCU_VARIANT == MCU_ESP32
		const int16_t pin_cs = 18;
		const int16_t pin_reset = 23;
		const int16_t pin_dio = 26;
		const int16_t pin_led_rx = 25;
		const int16_t pin_led_tx = 25;

		// TODO: Reset
		#define CONFIG_UART_BUFFER_SIZE 6144
		#define CONFIG_QUEUE_SIZE 6144
		#define CONFIG_QUEUE_MAX_LENGTH 250

		#define EEPROM_SIZE 4096
		#define EEPROM_OFFSET 0
    volatile uint8_t prefbuf[EEPROM_SIZE];
    volatile boolean prefbuf_update = false;
	#endif
  
  #ifdef ESP32
    #define eeprom_addr(a) (a)
  #else
	  #define eeprom_addr(a) (a+EEPROM_OFFSET)
  #endif
	// MCU independent configuration parameters
	const int32_t serial_baudrate  = 115200;
	const int16_t lora_rx_turnaround_ms = 50;

	// SX1276 RSSI offset to get dBm value from
	// packet RSSI register
	const int16_t  rssi_offset      = 157;

	// Default LoRa settings
	int16_t  lora_sf   	   = 7;
	int16_t  lora_cr       = 5;
	int16_t  lora_txp      = 0x00;
	uint32_t lora_bw   = 125000;
	uint32_t lora_freq = 868000000;

	// Operational variables
	bool radio_locked  = true;
	bool radio_online  = false;
	bool hw_ready      = false;
	bool promisc       = false;
	bool implicit      = false;
	uint8_t implicit_l = 0;

	uint8_t op_mode   = MODE_HOST;
	uint8_t model     = 0x00;
	uint8_t hwrev     = 0x00;
	
	int16_t		last_rssi		= -292;
	uint8_t last_rssi_raw   = 0x00;
	uint8_t last_snr_raw	= 0x00;
	size_t	read_len		= 0;
	uint8_t seq				= 0xFF;
	
	// Incoming packet buffer
	uint8_t pbuf[MTU];

	// KISS command buffer
	uint8_t cbuf[CMD_L];

	// LoRa transmit buffer
	uint8_t tbuf[MTU];

	uint32_t stat_rx		= 0;
	uint32_t stat_tx		= 0;

	bool stat_signal_detected = false;
	bool stat_signal_synced   = false;
	bool stat_rx_ongoing      = false;
	bool dcd				  = false;
	bool dcd_led              = false;
	bool dcd_waiting          = false;
	uint16_t dcd_count        = 0;
	uint16_t dcd_threshold    = 15;

	uint32_t status_interval_ms = 3;
	uint32_t last_status_update = 0;

	// Status flags
	const uint8_t SIG_DETECT = 0x01;
	const uint8_t SIG_SYNCED = 0x02;
	const uint8_t RX_ONGOING = 0x04;

#endif
