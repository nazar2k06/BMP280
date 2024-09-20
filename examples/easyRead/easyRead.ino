#include <BMP280.h>

float t, p;

BMP280 bmp;

void setup() {
  Serial.begin(9600);

  Serial.print("Begin: ");
  switch(bmp.begin()) {
  case 0:
    Serial.println("OK");
    break;
  case 1:
    Serial.println("Read request error");
    break;
  case 2:
    Serial.println("Receive error");
    break;
  case 3:
    Serial.println("CHIPID error");
    break;
  case 4:
    Serial.println("read calibration data error");
    break;
  case 5:
    Serial.println("write settings error");
    break;
  }

  Serial.print("Write default settings: ");
  switch(bmp.writeSettings()) {
  case 0:
    Serial.println("OK");
    break;
  case 1:
    Serial.println("write to REGISTER_CONFIG error");
    break;
  case 2:
    Serial.println("write to REGISTER_CONTROL error");
    break;
  }

  uint8_t error = bmp.writeSettings(BMP280_OVERSAMPLING::SAMPLING_X16,
                                    BMP280_OVERSAMPLING::SAMPLING_X16,
                                    BMP280_POWER_MODE::MODE_NORMAL,
                                    BMP280_STANDBY_DURATION::STANDBY_MS_1,
                                    BMP280_FILTER::FILTER_X16);
  Serial.print("Write settings: ");
  switch(error) {
  case 0:
    Serial.println("OK");
    break;
  case 1:
    Serial.println("write to REGISTER_CONFIG error");
    break;
  case 2:
    Serial.println("write to REGISTER_CONTROL error");
    break;
  }
}

void loop() {
  Serial.println("\n~~~~~~~~~~~");
  Serial.println("Status: ");

  switch (bmp.status()) {
  case 0:
    Serial.println("OK");
    break;
  case 1:
    Serial.println("Read request error");
    break;
  case 2:
    Serial.println("Receive error");
    break;
  case 3:
    Serial.println("CHIPID error");
    break;
  case 4:
    Serial.println("read calibration data error");
    break;
  case 5:
    Serial.println("write settings error");
    break;
  }

  Serial.print("\nRead: ");
  switch(bmp.read(&t, &p)) {
  case 0:
    Serial.println("OK");
    break;
  case 1:
    Serial.println("Temp & Pressure NULL error");
    break;
  case 2:
    Serial.println("Offline | write setting | read calibration data error");
    break;
  case 3:
    Serial.println("Read temp error");
    break;
  case 4:
    Serial.println("Read pressure error");
    break;
  case 5:
    Serial.println("Read pressure (division by zero) error");
    break;
  }
  Serial.println(String("T: ") + t);
  Serial.println(String("P: ") + p + " Pa");

  Serial.println(String("\nRead temp: ") + bmp.getT());
  Serial.println(String("Read pressure: ") + bmp.getP() + " Pa");
  Serial.println("~~~~~~~~~~~");
  
  delay(3000);
}
