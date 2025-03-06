#include <Arduino.h>
#include <SimpleFOC.h>
#include "main.h"

int SAFETY_PIN = 13; //pin safety switch
SensorState sv; //  allocated stack

float targetAngle = 0.0;

//------------------------------ENCODER--------------------------
Encoder encoder = Encoder(2, 3, 2048);


//----------------------------------MOTOR-----------------------
//SensorState sensorPointer = &sensorState; 
//8 is enable, these pin numbers are on the back of the motor driver corresponding to a,b,c
//BLDCDriver3PWM driver = BLDCDriver3PWM(3, 10, 11, 8);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);

// motor info!
// GM4108H
// https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217?srsltid=AfmBOoqxeW2Hb-j6NUhU7VB4WXt1Kw264wGX11YsdpgPQk_aYoP7Nqlo
// 22 poles, 11 poll pairs
// Resistance(Ω): 11.1Ω±5%
// Load Current (A): 1.5A

BLDCMotor motor = BLDCMotor(11);

// instantiate the commander
// press T followed by number to change motor speed in serial termal during program running
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doAngle(char* cmd) { command.scalar(&targetAngle, cmd); }

void setup() {
  //initialize struct variables
  sv.motorOn = true;
  sv.lastButtonState = HIGH;
  
  //initialize pins
  pinMode(SAFETY_PIN, INPUT);

  //initialize serial monitoring
  Serial.begin(115200);
  Serial.println("hello world!");
  SimpleFOCDebug::enable(&Serial);

  //encoder properties
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  
  //motor properties
  // // power supply voltage [V] from battery
  driver.voltage_power_supply = 11.1;
  // amps = voltage / ohms 
  // 11.1v battery / 11 ohms = 1amp which is well below 1.5 amps max
  // 11.1v as max volage is well within range
  driver.voltage_limit = 11.1;

  if(!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp

  //max current 1.3A for this motor
  //1.3A > 5 / 5.6
  motor.voltage_limit = 3;   // [V]
  motor.velocity_limit = 10;
   // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

   // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s] 
  //motor.target = 2; // one rotation per second

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('X', doAngle, "angle");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  // bool currentButtonState = digitalRead(SAFETY_PIN);
  // if (sv.lastButtonState == HIGH && currentButtonState == LOW) {
  //   // toggle motor state
  //   sv.motorOn = !sv.motorOn;
  // }
  // sv.lastButtonState = currentButtonState;
  //Serial.println(sv.motorOn);

  //updateStuff();
  encoder.update();
  float currentAngle = encoder.getAngle();
  float currentVelocity = encoder.getVelocity();
  Serial.print(currentAngle);
  Serial.print("\t");
  Serial.println();
  motor.move();

  // user communication
  command.run();
}

//lets me know arduino is on
// void blinkyLight() {
//   digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
//   delay(1000);                      // wait for a second
//   digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
//   delay(1000); 
// }

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}

void updateStuff() {
  encoder.update();
  float currentAngle = encoder.getAngle();
  float currentVelocity = encoder.getVelocity();
  Serial.print(currentAngle);
  Serial.print("\t");
  Serial.println();

  //if current angle is not equal to target angle, go to target angle
  //positive voltage is CCW
  //negative is CW
  if (abs(currentAngle - targetAngle) > 0.5) {
    if (targetAngle < currentAngle) {
      motor.target = -2;
    } else {
      motor.target = 2;
    }
  } else {
    motor.target = 0;
  }
}