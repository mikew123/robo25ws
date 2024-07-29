/**
 * PWM for servos and Throttle ESC uses this library
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Servo-RP2040/graphs/contributors.
 
 
 * Mike Williamson 7-5-2024 engine controller
 * Resides in the "engine compartment"
 * Connects to computer using USB serial
 * Selects RC receiver or computer control
 * Generates PWM signals for steering servo and Throttle ESC

 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <107-Arduino-Servo-RP2040.h>
#include <Arduino_JSON.h>
#include <strings.h>
#include <string>
//#include <cstdlib>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static _107_::Servo sSteer, sThrottle, sShift, pwm_3, muxSel;

#define pwmPerNominal 11000

#define PIN_sSteer 0
#define PIN_sThrottle 1
#define PIN_sShift 2
#define PIN_PWM3 3

#define PIN_SEL  4
#define SEL_RCVR 1000
#define SEL_COMP 2000

#define PIN_mSteer 5
#define PIN_mThrottle 6
#define PIN_mShift 7

// PWM values 
#define steerCenter 1575
#define steerRightMax 2000
#define steerLeftMax 1000

#define throttleZero 1500
#define throttleFwdMax 1900
#define throttleRevMax 1100
// These are impericaly measured, how do the change with temp?
#define throttleFwdMin 1545
#define throttleRevMin 1475

#define shiftLow 1100 // shift is a static value
#define shiftHigh 1900

#define systemStatusPeriod 1000
#define systemRxDataPeriod 50

#define failsafeThrottleThresh (throttleZero+100)

// Loop timers for precise interval periods
unsigned long systemStatusMillis_last = 0;
unsigned long systemRxDataMillis_last = 0;

unsigned long mSteer_micros_last0 = 0, mSteer_micros_last1 = 0;
bool mSteer_meas_rdy = 0;
int mSteer_per=0, mSteer_wid=0;


unsigned long mThrottle_micros_last0 = 0, mThrottle_micros_last1 = 0;
bool mThrottle_meas_rdy = 0;
int mThrottle_per=0, mThrottle_wid=0;

unsigned long mShift_micros_last0 = 0, mShift_micros_last1 = 0;
bool mShift_meas_rdy = 0;
int mShift_per=0, mShift_wid=0;

bool shiftState = 0;
bool muxSelRcvr = true;


bool failsafeActive = false;
bool receiverSignalsValid = false;


// Computer (slave mux input) signals
float sSteerPct = 0;
float sThrottlePct = 0;
String sShiftGear = "low";


String sFailSafe = "ena";
String sRcMux = "rcvr";
String sLoopback = "off";
bool sRxEnable = false;

/**************************************************************************************
 * INTERRUPTS
 **************************************************************************************/

// TODO: Handle wrap??
void pin_mSteer_interrupt (void) {
  unsigned long mSteer_micros = micros();
  if(digitalRead(PIN_mSteer)==0) {
    // HI->LO transition
    mSteer_per = mSteer_micros - mSteer_micros_last0;
    mSteer_wid = mSteer_micros - mSteer_micros_last1;
    mSteer_micros_last0 = mSteer_micros;
    mSteer_meas_rdy = 1;
  } else {
    // LO->HI transition
    mSteer_micros_last1 = mSteer_micros;
  }
}

// TODO: Handle wrap??
void pin_mThrottle_interrupt (void) {
  unsigned long mThrottle_micros = micros();
  if(digitalRead(PIN_mThrottle)==0) {
    // HI->LO transition
    mThrottle_per = mThrottle_micros - mThrottle_micros_last0;
    mThrottle_wid = mThrottle_micros - mThrottle_micros_last1;
    mThrottle_micros_last0 = mThrottle_micros;
    mThrottle_meas_rdy = 1;
  } else {
    // LO->HI transition
    mThrottle_micros_last1 = mThrottle_micros;
  }
}

// TODO: Handle wrap??
void pin_mShift_interrupt (void) {
  unsigned long mShift_micros = micros();
  if(digitalRead(PIN_mShift)==0) {
    // HI->LO transition
    mShift_per = mShift_micros - mShift_micros_last0;
    mShift_wid = mShift_micros - mShift_micros_last1;
    mShift_micros_last0 = mShift_micros;
    mShift_meas_rdy = 1;
  } else {
    // LO->HI transition
    mShift_micros_last1 = mShift_micros;
  }
}

unsigned long steerPct2Wid(float steerPct) {
  int steerPwmOffset = 0;
  if(sSteerPct>0) {
    steerPwmOffset = (+steerPct/100.0)*(steerRightMax - steerCenter);
  } else {
    steerPwmOffset = (-steerPct/100.0)*(steerLeftMax - steerCenter);
  }
  int steerPwm = steerCenter + steerPwmOffset;
  return(steerPwm);
}
unsigned long throttlePct2Wid(float throttlePct) {
    int throttlePwmOffset = 0;
    int throttlePwm = throttleZero;
    if(throttlePct>0) {
      throttlePwmOffset = (+throttlePct/100.0)*(throttleFwdMax - throttleFwdMin);
      throttlePwm = throttleFwdMin + throttlePwmOffset;
    } else {
      throttlePwmOffset = (-throttlePct/100.0)*(throttleRevMax - throttleRevMin);
      throttlePwm = throttleRevMin + throttlePwmOffset;
    }
    return(throttlePwm);
}
float throttleWid2Pct(unsigned long throttleWid) {
  // Throttle has a "dead zone" - pct=0 in dead zone
  float throttlePct = 0.0;
  if(throttleWid > throttleFwdMin) {
    throttlePct = (100.0*(throttleWid - throttleFwdMin))/(throttleFwdMax - throttleFwdMin);
  } else if(throttleWid < throttleRevMin) {
    throttlePct = (-100.0*(throttleRevMin - throttleWid))/(throttleRevMin - throttleRevMax);
  }
  return(throttlePct);
}
float steerWid2Pct(unsigned long steerWid) {
  float steerPct = 0.0;
  if(steerWid > steerCenter) {
    steerPct = (100.0*(steerWid - steerCenter))/(steerRightMax - steerCenter);
  } else if(steerWid < steerCenter) {
    steerPct = (-100.0*(steerCenter - steerWid))/(steerCenter - steerLeftMax);
  }
  return(steerPct);
}

/**************************************************************************************
 * SETUP
 **************************************************************************************/

void setup()
{
  Serial.begin(1000000);
  while (!Serial) { }

  // PWM outputs
  sSteer.attach(PIN_sSteer);
  sThrottle.attach(PIN_sThrottle);
  sShift.attach(PIN_sShift);
  pwm_3.attach(PIN_PWM3);
  muxSel.attach(PIN_SEL);
  // Select RC receiver as default
  muxSel.writeMicroseconds(SEL_RCVR);
  // Set default PWM signals on slave signals to RC MUX
  sSteer.writeMicroseconds(steerCenter);
  sThrottle.writeMicroseconds(throttleZero);
  sShift.writeMicroseconds(shiftLow);
  pwm_3.writeMicroseconds(1500);

  // PWM inputs
  pinMode(PIN_mSteer, INPUT_PULLUP);
  pinMode(PIN_mThrottle, INPUT_PULLUP);
  pinMode(PIN_mShift, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mThrottle), pin_mThrottle_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;

  Serial.println("Engine controller started");
}

/**************************************************************************************
 * LOOP
 **************************************************************************************/

void loop()
{
  unsigned long loopMillis = millis();

  // impliment failsafe mechanism
  checkFailsafe();

  // process any serial receive data
  serialRx();
  
  // Decode mux selection
  muxSelectDecode();

  // computer signals to servos and motor
  computerSignals();

  // send a system status message
  sendSystemStatus(loopMillis);

  // send receiver RX data message
  sendRxData(loopMillis);

  delay(0); // TODO: delay 0 ???
}


// computer signals to servos and motor

void computerSignals() {
  // loopback operation
  if (sLoopback == "wid") {
    // loopback signal widths
    sSteer.writeMicroseconds(mSteer_wid);
    sThrottle.writeMicroseconds(mThrottle_wid);
  }
  else if (sLoopback == "pct") {
    // loopback percentages
    sSteer.writeMicroseconds(steerPct2Wid(steerWid2Pct(mSteer_wid)));
    sThrottle.writeMicroseconds(throttlePct2Wid(throttleWid2Pct(mThrottle_wid)));
  }
  // normal operation
  else if ((muxSelRcvr==false) && (failsafeActive==false) ) {
    // send computer signals to pwm mux
    // Convert percent signals to PWM widths
    int steerPwmOffset = 0;
    if(sSteerPct>0) {
      steerPwmOffset = (+sSteerPct/100.0)*(steerRightMax - steerCenter);
    } else {
      steerPwmOffset = (-sSteerPct/100.0)*(steerLeftMax - steerCenter);
    }
    int steerPwm = steerCenter + steerPwmOffset;
    sSteer.writeMicroseconds(steerPwm);

    int throttlePwmOffset = 0;
    int throttlePwm = throttleZero;
    if(sThrottlePct>0) {
      throttlePwmOffset = (+sThrottlePct/100.0)*(throttleFwdMax - throttleFwdMin);
      throttlePwm = throttleFwdMin + throttlePwmOffset;
    } else {
      throttlePwmOffset = (-sThrottlePct/100.0)*(throttleRevMax - throttleRevMin);
      throttlePwm = throttleRevMin + throttlePwmOffset;
    }
    sThrottle.writeMicroseconds(throttlePwm);

    int shiftGearPwm = shiftLow;
    if(sShiftGear=="high") {
      shiftGearPwm = shiftHigh;
    } else {
      shiftGearPwm = shiftLow;
    }
    sShift.writeMicroseconds(shiftGearPwm);

  } else {
    // send idle signals to pwm mux
    sSteer.writeMicroseconds(steerCenter);
    sThrottle.writeMicroseconds(throttleZero);
    sShift.writeMicroseconds(shiftLow);
  }
}


// Impliment failsafe
// When computer is selected the transmitter trigger must be pulled to operate
// If the trigger is not pulled then the failsafe is activated and the signals
// to the servos and motor become the default values

// Monitor failsafe mechanism
void checkFailsafe() {
  if ((muxSelRcvr == true) || (sFailSafe == "dis")) {
    failsafeActive = false;
  } else  {
    // computer is selected for control
    if (mThrottle_wid < failsafeThrottleThresh) {
      // Failsafe active when throttle on transmitter is released
      failsafeActive = true;
    } else {
      failsafeActive = false;
    }
  }
}


// Send receiver RX data to host computer
void sendRxData(unsigned long loopMillis) {
  if (sRxEnable == false) return;

  // Send system status at defined rate
  // Use unsigned long long to cause number wrap to work
  if (((signed long long)loopMillis - systemRxDataMillis_last) > systemRxDataPeriod) {
    systemRxDataMillis_last = loopMillis;
    
    JSONVar myObject;

    myObject["rx"]["sft"] = shiftState==0?"low":"high";
    myObject["rx"]["thr"] = int(throttleWid2Pct(mThrottle_wid)*10)/10.0;
    myObject["rx"]["str"] = int(steerWid2Pct(mSteer_wid)*10)/10.0;

    String jsonString = JSON.stringify(myObject);
    Serial.println(jsonString);   

  }
  
}

// Send system status message to host computer
void sendSystemStatus(unsigned long loopMillis) {
  // Send system status at defined rate
  if (((signed long long)loopMillis - systemStatusMillis_last) > systemStatusPeriod) {
    systemStatusMillis_last = loopMillis;

    // DEBUG print servos PWM
    //monitorReceiver();
  
    JSONVar myObject;
  
    if(shiftState==0) {
      // low gear
      myObject["systat"]["sft"] = "low";
    } else {
      // high gear
      myObject["systat"]["sft"] = "hi";
    }
  
    if(muxSelRcvr) {
      myObject["systat"]["mux"] = "rcvr";
    } else {
      myObject["systat"]["mux"] = "comp";
    }
  
    if(failsafeActive) {
      myObject["systat"]["fsa"] = true;
    } else {
      myObject["systat"]["fsa"] = false;
    }
  
    if(receiverSignalsValid) {
      myObject["systat"]["rca"] = true;
    } else {
      myObject["systat"]["rca"] = false;
    }
  
  
    String jsonString = JSON.stringify(myObject);
    Serial.println(jsonString);
  
  }

}

void checkTransmitterActive() {

  // TODO: This does not work after transmitter 1st powered on
  //       Maybe monitor period variation which seems small when transmitter
  //       is powered on. But probably track an average nominal instead of
  //       fixed nominal to compensate for temp and age etc
  //       Also check receiver signal timeout 
  // check for active valid receiver signals
  // verify that PWM period is within 10%
  float mSteerPctOffset = 100.0*(mSteer_per-pwmPerNominal)/pwmPerNominal;
  //Serial.println(mSteerPctOffset);
  if (   (fabs(mSteerPctOffset) < 10)) {
    receiverSignalsValid = true;
  } else {
    receiverSignalsValid = false;
  }
}

void monitorReceiver() {

  // TODO: timeout
  // Monitor PWM signals from RC receiver on master MUX inputs
  if(mSteer_meas_rdy == 1){
    Serial.print("mSteer:");
    Serial.print(mSteer_per);
    Serial.print(",");
    Serial.println(mSteer_wid);
    mSteer_meas_rdy = 0;
  }

  if(mThrottle_meas_rdy == 1){
    Serial.print("mThrottle:");
    Serial.print(mThrottle_per);
    Serial.print(",");
    Serial.println(mThrottle_wid);
    mThrottle_meas_rdy = 0;
  }

  if(mShift_meas_rdy == 1){
    Serial.print("mShift:");
    Serial.print(mShift_per);
    Serial.print(",");
    Serial.println(mShift_wid);
    mShift_meas_rdy = 0;

  }
}

// Select receiver or computer using steering and shift and throttle
// The throttle must be center +- 10% to change selection
// Use shift high >1500+10%, low <1500-10%
// Select computer when steering > center+10% and shift high to low
// Select receiver when steering < center-10% and shift high to low
void muxSelectDecode() {
  if(fabs(mThrottle_wid-throttleZero) < (throttleZero*0.1)) {
    if(mShift_wid < 1500*0.9) {
      if(shiftState==1) {
        if(mSteer_wid > steerCenter*1.05) muxSelRcvr = true;
        else muxSelRcvr = false;
      }
      shiftState = 0;
    }
    if(mShift_wid > 1500*1.1) {
      shiftState = 1;
    }
  }

  // TODO: move to a pwm output module
  muxSel.writeMicroseconds(muxSelRcvr?SEL_RCVR:SEL_COMP);

}

void serialRx() {
  // Check serial port for a JSON message
  if (Serial.available() > 0) {
    // read the incoming string and parse it
    String incomingString = Serial.readStringUntil('\n');
    jsonParse(incomingString.c_str());
  }
}
bool jsonParseDrv(JSONVar drvObject) {

    if (drvObject.hasOwnProperty("str")) {
      sSteerPct = (double) drvObject["str"];
      //Serial.print("steer = ");
      //Serial.println(sSteerPct);
    }
  
    if (drvObject.hasOwnProperty("thr")) {
      sThrottlePct = (double) drvObject["thr"];
      //Serial.print("throttle = ");
      //Serial.println(sThrottlePct);
    }
  
    if (drvObject.hasOwnProperty("sft")) {
      sShiftGear = (String) drvObject["sft"];
      //Serial.print("gear = ");
      //Serial.println(sShiftGear);
    }

  return true;
}

bool jsonParseCfg(JSONVar cfgObject) {
    // Failsafe 
    if (cfgObject.hasOwnProperty("fsa")) {
      sFailSafe = (String) cfgObject["fsa"];
      //Serial.print("fail safe = ");
      //Serial.println(sFailSafe);
    }
  
    // RC mux tx/cpu control, tx PWM decode shares mode
    if (cfgObject.hasOwnProperty("mux")) {
      sRcMux = (String) cfgObject["mux"];
      //Serial.print("RC mux = ");
      //Serial.println(sRcMux);
    }
  
    // Loopbacks
    if (cfgObject.hasOwnProperty("lbk")) {
      sLoopback = (String) cfgObject["lbk"];
      //Serial.print("loop back = ");
      //Serial.println(sLoopback);
    }
  
    // receiver RX msg enable
    if (cfgObject.hasOwnProperty("rxe")) {
      sRxEnable = (bool) cfgObject["rxe"];
      // Serial.print("rx enable = ");
      // Serial.println(sRxEnable);
    }

  return true;
}

bool jsonParse(const char *jsonStr) {
  //Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing JSON string input failed!");
    return false;
  }

  if (myObject.hasOwnProperty("drv")) {
    return jsonParseDrv(myObject["drv"]);
  }
  
  if (myObject.hasOwnProperty("cfg")) {
    return jsonParseCfg(myObject["cfg"]);
  }

  return false;
}
