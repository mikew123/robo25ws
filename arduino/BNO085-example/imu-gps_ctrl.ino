#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Arduino_JSON.h>
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SERIAL myGNSS;

// No reset pin for UART
#define BNO08X_RESET -1

// UART0/Serial1 pins
#define TX0 0
#define RX0 1

// UART1/Serial2 pins
#define TX1 4
#define RX1 5

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Global variables

bool g_imuEna = false;
bool g_gpsEna = false;
bool g_cpsEna = false;

int g_gpsInterval = 100;

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

void setupImu(void) {
  Serial1.setTX(TX0);
  Serial1.setRX(RX0);
  Serial1.setFIFOSize(1024);

  if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Enable sensors 
  setReports();
}

void setupGps(void) {
  Serial2.setTX(TX1);
  Serial2.setRX(RX1);
  
  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 115200 baud.
  //Loop until we're in sync and then ensure it's at 115200 baud.
  do {
    Serial2.println("GNSS: trying 115200 baud");
    Serial2.begin(115200);
    if (myGNSS.begin(Serial2) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial2.begin(9600);
    if (myGNSS.begin(Serial2) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 115200");
        myGNSS.setSerialRate(115200);
        delay(100);
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
    } else {
        myGNSS.factoryDefault();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setMeasurementRate(g_gpsInterval);
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     //  pause  until serial console opens

  Serial.println("IMU + GPS");
  
  setupImu();

  setupGps();

  delay(10);
}


void loop() {
  serialRx();
  procImu();
  procGps();
}


void serialRx() {
  // Check serial port for a JSON message
  if (Serial.available() > 0) {
    // read the incoming string and parse it
    String incomingString = Serial.readStringUntil('\n');
    jsonParse(incomingString.c_str());
  }
}

bool jsonParse(const char *jsonStr) {
  //Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing JSON string input failed!");
    return false;
  }

  if (myObject.hasOwnProperty("cfg")) {
    return jsonParseCfg(myObject["cfg"]);
  }

  return false;
}

bool jsonParseCfg(JSONVar cfgObject) {
  bool retVal = false;
  if (cfgObject.hasOwnProperty("imu")) {
    g_imuEna = (bool) cfgObject["imu"];
    retVal = true;
  }
  if (cfgObject.hasOwnProperty("gps")) {
    g_gpsEna = (bool) cfgObject["gps"];
    retVal = true;
  }
  if (cfgObject.hasOwnProperty("cps")) {
    g_cpsEna = (bool) cfgObject["cps"];
    retVal = true;
  }
  return retVal;
}

// process the serial port connected to the uBlox GNSS M10Q
// NOTE: default is 1 msg/sec and 9600 baud
void procGps() {
  if (g_gpsEna == false) return;

  static unsigned long lastTime = 0;

  //Query module 10 per second.
  //The module (blocking) only responds when a new position is available
  unsigned long currTime = millis();
  if (currTime - lastTime > g_gpsInterval)
  {
    lastTime = currTime;

    JSONVar jsonObject;
    jsonObject["gps"]["lat"] = myGNSS.getLatitude();
    jsonObject["imu"]["lon"] = myGNSS.getLongitude();
    jsonObject["imu"]["alt"] = myGNSS.getAltitude();
    jsonObject["imu"]["siv"] = myGNSS.getSIV();
    Serial.println(jsonObject);

  }
}

void procImu(void) {
  if (g_imuEna == false) return;

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

