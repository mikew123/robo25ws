#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Arduino_JSON.h>

// No reset pin for UART
#define BNO08X_RESET -1

// UART0/Serial1 pins
#define TX0 0
#define RX0 1

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setReports(void) {
  uint32_t period = 100000; // 10 Hz
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, period)) { //0x02
    Serial.print("Could not enable report SH2_GYROSCOPE_CALIBRATED 0x");
    Serial.println(SH2_GYROSCOPE_CALIBRATED,HEX);
  }
  if (! bno08x.enableReport(SH2_LINEAR_ACCELERATION, period)) { //0x04
    Serial.print("Could not enable report SH2_LINEAR_ACCELERATION 0x");
    Serial.println(SH2_LINEAR_ACCELERATION,HEX);
  }
  if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV, period)) { //0x28
    Serial.print("Could not enable report SH2_ARVR_STABILIZED_RV 0x");
    Serial.println(SH2_ARVR_STABILIZED_RV,HEX);
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");
    
  Serial1.setTX(TX0);
  Serial1.setRX(RX0);
  Serial1.setFIFOSize(1024);

  if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
//  Serial.println("BNO08x Found!");

  // Enable sensors 
  setReports();

 // Serial.println("Reading events");
  delay(10);
}


void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GYROSCOPE_CALIBRATED: //0x02
        json_SH2_GYROSCOPE_CALIBRATED(sensorValue);
        break;
      case SH2_LINEAR_ACCELERATION:  //0x04
        json_SH2_LINEAR_ACCELERATION(sensorValue);
        break;
      case SH2_ARVR_STABILIZED_RV:   // 0x28
        json_SH2_ARVR_STABILIZED_RV(sensorValue);
        break;
    }
  }
}

void json_SH2_GYROSCOPE_CALIBRATED(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["rvel"]["seq"]  = sensorValue.sequence;
  jsonObject["imu"]["rvel"]["stat"] = sensorValue.status;
  jsonObject["imu"]["rvel"]["x"]    = sensorValue.un.gyroscope.z;
  jsonObject["imu"]["rvel"]["y"]    = sensorValue.un.gyroscope.y;
  jsonObject["imu"]["rvel"]["z"]    = sensorValue.un.gyroscope.x;
  Serial.println(jsonObject);
}

void json_SH2_LINEAR_ACCELERATION(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["lacc"]["seq"] = sensorValue.sequence;
  jsonObject["imu"]["lacc"]["stat"] = sensorValue.status;
  jsonObject["imu"]["lacc"]["x"] = sensorValue.un.linearAcceleration.x;
  jsonObject["imu"]["lacc"]["y"] = sensorValue.un.linearAcceleration.y;
  jsonObject["imu"]["lacc"]["z"] = sensorValue.un.linearAcceleration.z;
  Serial.println(jsonObject);
}

void json_SH2_ARVR_STABILIZED_RV(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["rvec"]["seq"] = sensorValue.sequence;
  jsonObject["imu"]["rvec"]["stat"] = sensorValue.status;
  jsonObject["imu"]["rvec"]["i"] = sensorValue.un.arvrStabilizedRV.i;
  jsonObject["imu"]["rvec"]["j"] = sensorValue.un.arvrStabilizedRV.j;
  jsonObject["imu"]["rvec"]["k"] = sensorValue.un.arvrStabilizedRV.k;
  jsonObject["imu"]["rvec"]["real"] = sensorValue.un.arvrStabilizedRV.real;
  Serial.println(jsonObject);
}

