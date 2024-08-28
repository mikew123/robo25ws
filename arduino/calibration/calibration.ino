/*
QMC5883LCompass.h Library Calibration Example Sketch
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Upload this calibration sketch onto your Arduino to provide calibration for your QMC5883L chip.
After upload, run the serial monitor and follow the directions.
When prompted, copy the last line into your project's actual sketch.

===============================================================================================================
Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]
===============================================================================================================











                                          UTILITY SKETCH
                                    NO SERVICABLE PARTS BELOW












*/
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

// Wire I2C0 pins
#define SDA0 8
#define SCL0 9

void setup() {
  Serial.begin(9600);
  
  Wire.setSDA(SDA0);
  Wire.setSCL(SCL0);
  
  compass.init();
  
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  Serial.println();
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2));
  Serial.println(");");
  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2));
  Serial.println(");");
}

void loop() {
  delay(1000);
}

/*
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(-121.00, -114.00, 610.00);
compass.setCalibrationScales(2.33, 2.47, 0.46);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(696.00, 443.00, -364.00);
compass.setCalibrationScales(0.94, 0.95, 1.13);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(258.00, 134.00, -396.00);
compass.setCalibrationScales(0.92, 0.91, 1.22);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(39.00, 333.00, 830.00);
compass.setCalibrationScales(1.00, 0.82, 1.27);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(405.00, 129.00, 42.00);
compass.setCalibrationScales(1.05, 0.98, 0.97);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(505.00, 259.00, 169.00);
compass.setCalibrationScales(1.15, 0.91, 0.97);
CALIBRATING. Keep moving your sensor...
DONE. Copy the lines below and paste it into your projects sketch.);

compass.setCalibrationOffsets(1017.00, 155.00, 261.00);
compass.setCalibrationScales(1.34, 0.85, 0.93);
*/
