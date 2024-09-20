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

#include "BMP280.h"

/* --- constuctor --- */
BMP280::BMP280() {
	Wire.begin();

	memset(&calibration_data, 0, sizeof(calibration_data));

	last_settings.oversampling_T = BMP280_OVERSAMPLING::SAMPLING_X16;
	last_settings.oversampling_P = BMP280_OVERSAMPLING::SAMPLING_X16;
	last_settings.power_mode = BMP280_POWER_MODE::MODE_NORMAL;
	last_settings.standby_duration = BMP280_STANDBY_DURATION::STANDBY_MS_1;
	last_settings.filter = BMP280_FILTER::FILTER_X2;

	offline_flag = true;
	read_calibration_data_flag = false;
	write_settings_flag = false;
}

/* --- begin --- */
uint8_t BMP280::begin() {
	return status();
}

/* --- status --- */
uint8_t BMP280::status() {
	uint8_t chipid_read = 0;
	uint8_t err_code = read_data(BMP280_REGISTER_CHIPID, &chipid_read, 1);

	if (err_code) {
		offline_flag = true;

		return err_code;
	}

	if (chipid_read != BMP280_CHIPID) {
		offline_flag = true;

		return 3;
	}

	if (!read_calibration_data_flag || offline_flag) {
		read_calibration_data();

		if (!read_calibration_data_flag) {
			return 4;
		}
	}

	if (!write_settings_flag || offline_flag) {
		writeSettings();

		if (!write_settings_flag) {
			return 5;
		}
	}

	offline_flag = false;
	return 0;
}


/* --- write settings --- */
uint8_t BMP280::writeSettings() {
	return writeSettings(last_settings.oversampling_T,
						  last_settings.oversampling_P,
						  last_settings.power_mode,
						  last_settings.standby_duration,
						  last_settings.filter);
}

/* --- write settings --- */
uint8_t BMP280::writeSettings(BMP280_OVERSAMPLING oversampling_T,
							  BMP280_OVERSAMPLING oversampling_P,
							  BMP280_POWER_MODE power_mode,
							  BMP280_STANDBY_DURATION standby_duration,
							  BMP280_FILTER filter) {
	uint8_t register_config = 0;
	uint8_t register_control = 0;
	write_settings_flag = false;

	register_config = (standby_duration << 5) | (filter << 2) | 0x01;
	register_control = (oversampling_T << 5) | (oversampling_P << 2) | power_mode;

	if (write_u8(BMP280_REGISTER_CONFIG, register_config)) {
		return 1;
	}
	if (write_u8(BMP280_REGISTER_CONTROL, register_control)) {
		return 2;
	}

	last_settings.oversampling_T = oversampling_T;
	last_settings.oversampling_P = oversampling_P;
	last_settings.power_mode = power_mode;
	last_settings.standby_duration = standby_duration;
	last_settings.filter = filter;

	write_settings_flag = true;
	return 0;
}


/* --- main read function --- */
uint8_t BMP280::read(float* temp, float* press) {
	int32_t t_fine = 0;
	int32_t adc_T;
	int32_t var1, var2;

	if (temp == NULL && press == NULL) {
		return 1;
	}

	status();
	if (!read_calibration_data_flag || !write_settings_flag || offline_flag) {
		return 2;
	}

	adc_T = read_u24(BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;
	if (!adc_T) {
		return 3;
	}

	var1 = ((((adc_T >> 3) - ((int32_t)calibration_data.dig_T1 << 1))) *
		((int32_t)calibration_data.dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t)calibration_data.dig_T1)) *
		((adc_T >> 4) - ((int32_t)calibration_data.dig_T1))) >> 12) *
		((int32_t)calibration_data.dig_T3)) >> 14;

	t_fine = var1 + var2;

	if (temp != NULL) {
		*temp = (t_fine * 5 + 128) >> 8;
		*temp /= 100;
	}

	if (press != NULL) {
		int32_t adc_P;
		int64_t var1, var2, p_read;

		adc_P = read_u24(BMP280_REGISTER_PRESSUREDATA);
		adc_P >>= 4;
		if (!adc_P) {
			return 4;
		}

		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)calibration_data.dig_P6;
		var2 = var2 + ((var1 * (int64_t)calibration_data.dig_P5) << 17);
		var2 = var2 + (((int64_t)calibration_data.dig_P4) << 35);
		var1 = ((var1 * var1 * (int64_t)calibration_data.dig_P3) >> 8) +
			((var1 * (int64_t)calibration_data.dig_P2) << 12);
		var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration_data.dig_P1) >> 33;

		if (var1 == 0) {
			return 5; // avoid exception caused by division by zero
		}
		p_read = 1048576 - adc_P;
		p_read = (((p_read << 31) - var2) * 3125) / var1;
		var1 = (((int64_t)calibration_data.dig_P9) * (p_read >> 13) * (p_read >> 13)) >> 25;
		var2 = (((int64_t)calibration_data.dig_P8) * p_read) >> 19;

		p_read = ((p_read + var1 + var2) >> 8) + (((int64_t)calibration_data.dig_P7) << 4);
		*press = p_read / 256.0;
	}

	return 0;
}

/* --- read temperature function --- */
float BMP280::getT() {
	float temp_read = 0;
	
	read(&temp_read, NULL);

	return temp_read;
}

/* --- read pressure function --- */
float BMP280::getP() {
	float press_read = 0;

	read(NULL, &press_read);

	return press_read;
}


/* --- read calibration data function --- */
bool BMP280::read_calibration_data() {
	calibration_data.dig_T1 = read_u16_rev(BMP280_REGISTER_DIG_T1);
	calibration_data.dig_T2 = read_i16_rev(BMP280_REGISTER_DIG_T2);
	calibration_data.dig_T3 = read_i16_rev(BMP280_REGISTER_DIG_T3);

	calibration_data.dig_P1 = read_u16_rev(BMP280_REGISTER_DIG_P1);
	calibration_data.dig_P2 = read_i16_rev(BMP280_REGISTER_DIG_P2);
	calibration_data.dig_P3 = read_i16_rev(BMP280_REGISTER_DIG_P3);
	calibration_data.dig_P4 = read_i16_rev(BMP280_REGISTER_DIG_P4);
	calibration_data.dig_P5 = read_i16_rev(BMP280_REGISTER_DIG_P5);
	calibration_data.dig_P6 = read_i16_rev(BMP280_REGISTER_DIG_P6);
	calibration_data.dig_P7 = read_i16_rev(BMP280_REGISTER_DIG_P7);
	calibration_data.dig_P8 = read_i16_rev(BMP280_REGISTER_DIG_P8);
	calibration_data.dig_P9 = read_i16_rev(BMP280_REGISTER_DIG_P9);

	read_calibration_data_flag = (calibration_data.dig_T1) ? true : false;
	return read_calibration_data_flag;
}


/* --- read data from device function --- */
uint8_t BMP280::read_data(uint8_t reg, uint8_t* pointer, uint8_t len) {
	Wire.beginTransmission(BMP280_DEVICE_ADDRESS);
	Wire.write(reg);

	if (Wire.endTransmission()) {
		return 1;
	}

	Wire.requestFrom((int) BMP280_DEVICE_ADDRESS, (int) len);
	if (!Wire.available()) {
		return 2;
	}

	while (Wire.available() && pointer != NULL) {
		*pointer = Wire.read();
		pointer++;
	}

	return 0;
}

/* --- read uint8_t from device function --- */
uint8_t BMP280::read_u8(uint8_t reg) {
	uint8_t data = 0;
	
	read_data(reg, &data, 1);

	return data;
}

/* --- read uint16_t from device function --- */
uint16_t BMP280::read_u16(uint8_t reg) {
	uint8_t bytes[2] = { 0, 0 };

	read_data(reg, bytes, 2);

	return ((uint16_t) bytes[0] << 8) | bytes[1];
}

/* --- read reversed uint16_t from device function --- */
uint16_t BMP280::read_u16_rev(uint8_t reg) {
	uint8_t bytes[2] = { 0, 0 };

	read_data(reg, bytes, 2);

	return ((uint16_t) bytes[1] << 8) | bytes[0];
}

/* --- read reversed int16_t from device function --- */
int16_t BMP280::read_i16_rev(uint8_t reg) {
	return (int16_t) read_u16_rev(reg);
}

/* --- read uint24_t from device function --- */
uint32_t BMP280::read_u24(uint8_t reg) {
	uint8_t bytes[3] = { 0, 0, 0};
	uint32_t data = 0;

	read_data(reg, bytes, 3);
	data = bytes[0];
	data <<= 8;
	data |= bytes[1];
	data <<= 8;
	data |= bytes[2];

	return data;
}


/* --- write uint8_t to device function --- */
uint8_t BMP280::write_u8(uint8_t reg, uint8_t data) {
	Wire.beginTransmission(BMP280_DEVICE_ADDRESS);
	Wire.write(reg);
	Wire.write(data);

	if (Wire.endTransmission()) {
		return 1;
	}

	return 0;
}
