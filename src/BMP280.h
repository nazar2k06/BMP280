/*
 * Library: BMP280
 *
 * Description: Short library that reads temperature and pressure.
 *
 * Author: Vereshchynskyi Nazar
 * Email: verechnazar12@gmail.com
 * Version: 1.0.0
 * Date: 13.04.2024
 */

#ifndef _BMP280_H
#define _BMP280_H

#include <Arduino.h>
#include <Wire.h>

/* --- device address & chip id --- */
#define BMP280_DEVICE_ADDRESS 0x76
#define BMP280_CHIPID 0x58

/* --- memory addresses --- */
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

/* --- BMP280 OVERSAMPLING SETTINGS --- */
enum BMP280_OVERSAMPLING {
	/** No over-sampling. */
	SAMPLING_NONE = 0x00,
	/** 1x over-sampling. */
	SAMPLING_X1 = 0x01,
	/** 2x over-sampling. */
	SAMPLING_X2 = 0x02,
	/** 4x over-sampling. */
	SAMPLING_X4 = 0x03,
	/** 8x over-sampling. */
	SAMPLING_X8 = 0x04,
	/** 16x over-sampling. */
	SAMPLING_X16 = 0x05
};

/* --- BMP280 POWER MODE SETTINGS --- */
enum BMP280_POWER_MODE {
	/** Sleep mode. */
	MODE_SLEEP = 0x00,
	/** Forced mode. */
	MODE_FORCED = 0x01,
	/** Normal mode. */
	MODE_NORMAL = 0x03,
	/** Software reset. */
	RESET_CODE = 0xB6
};

/* --- BMP280 STANDBY DURATION SETTINGS --- */
enum BMP280_STANDBY_DURATION {
	/** 1 ms standby. */
	STANDBY_MS_1 = 0x00,
	/** 62.5 ms standby. */
	STANDBY_MS_63 = 0x01,
	/** 125 ms standby. */
	STANDBY_MS_125 = 0x02,
	/** 250 ms standby. */
	STANDBY_MS_250 = 0x03,
	/** 500 ms standby. */
	STANDBY_MS_500 = 0x04,
	/** 1000 ms standby. */
	STANDBY_MS_1000 = 0x05,
	/** 2000 ms standby. */
	STANDBY_MS_2000 = 0x06,
	/** 4000 ms standby. */
	STANDBY_MS_4000 = 0x07
};

/* --- BMP280 FILTER SETTINGS --- */
enum BMP280_FILTER {
	/** No filtering. */
	FILTER_OFF = 0x00,
	/** 2x filtering. */
	FILTER_X2 = 0x01,
	/** 4x filtering. */
	FILTER_X4 = 0x02,
	/** 8x filtering. */
	FILTER_X8 = 0x03,
	/** 16x filtering. */
	FILTER_X16 = 0x04
};

/* --- all settings template --- */
struct Settings {
	BMP280_OVERSAMPLING oversampling_T;
	BMP280_OVERSAMPLING oversampling_P;
	BMP280_POWER_MODE power_mode;
	BMP280_STANDBY_DURATION standby_duration;
	BMP280_FILTER filter;
};

/* --- calibration data template --- */
struct Calibration_data {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
};

/* --- main class --- */
class BMP280 {
public:
	BMP280();
	uint8_t begin();
	uint8_t status();

	uint8_t writeSettings();
	uint8_t writeSettings(BMP280_OVERSAMPLING, BMP280_OVERSAMPLING, BMP280_POWER_MODE,
						   BMP280_STANDBY_DURATION, BMP280_FILTER);

	uint8_t read(float*, float*);
	float getT();
	float getP();

private:
	bool read_calibration_data();

	uint8_t read_data(uint8_t, uint8_t*, uint8_t);
	uint8_t read_u8(uint8_t);
	uint16_t read_u16(uint8_t);
	uint16_t read_u16_rev(uint8_t);
	int16_t read_i16_rev(uint8_t);
	uint32_t read_u24(uint8_t);

	uint8_t write_u8(uint8_t, uint8_t);

	Calibration_data calibration_data;
	Settings last_settings;

	bool offline_flag;
	bool read_calibration_data_flag;
	bool write_settings_flag;

};

#endif