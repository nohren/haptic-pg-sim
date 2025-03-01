#include <Arduino.h>
#include <SimpleFOC.h>
#include "main.h"

int SAFETY_PIN = 13; //pin safety switch
int PWM_A_1 = 5;
int PWM_B_1 = 9;
int PWM_C_1 = 6;
int ENABLE_1 = 8;
int ENCODER_A_1 = 2;
int ENCODER_B_1 = 3;
int PPR = 2048;
SensorState sv; //  allocated stack


//------------------------------ENCODER--------------------------
Encoder encoder = Encoder(ENCODER_A_1, ENCODER_B_1, PPR);


//----------------------------------MOTOR-----------------------
//SensorState sensorPointer = &sensorState; 
//8 is enable, these pin numbers are on the back of the motor driver corresponding to a,b,c
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_A_1, PWM_B_1, PWM_C_1, ENABLE_1);

// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain, don't make this too high
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 20, A0, A1, A2);

// motor info!
// GM4108H
// https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217?srsltid=AfmBOoqxeW2Hb-j6NUhU7VB4WXt1Kw264wGX11YsdpgPQk_aYoP7Nqlo
// 22 poles, 11 poll pairs
// Resistance(Ω): 11.1Ω±5%
// Load Current (A): 1.5A

BLDCMotor motor = BLDCMotor(11);

// instantiate the commander - remove to save memory
// press T followed by number to change motor speed in serial termal during program running
//Commander command = Commander(Serial);
//FOR OPEN LOOP
// void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
//void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  //initialize struct variables
  sv.motorOn = true;
  sv.lastButtonState = HIGH;
  
  //initialize pins
  pinMode(SAFETY_PIN, INPUT);

  //initialize serial monitoring
  Serial.begin(115200);
  //Serial.println("hello world!");
  SimpleFOCDebug::enable(&Serial);

  //encoder properties
  encoder.init();
  encoder.enableInterrupts(doA, doB);   // hardware interrupt enable
  // Quadrature mode enabling and disabling
  //  Quadrature::ON - CPR = 4xPPR  - default
  //  Quadrature::OFF - CPR = PPR
  // encoder.quadrature = Quadrature::OFF;

  motor.linkSensor(&encoder); // link the motor to the sensor
  
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
  motor.linkDriver(&driver);
  // link driver to cs
  current_sense.linkDriver(&driver);

  //current sense init hardware
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }
  
  motor.linkCurrentSense(&current_sense); // link the current sense to the motor

  motor.voltage_sensor_align = 15;   // aligning voltage
  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::foc_current; 
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);
  //motor.monitor_downsampling = 100; // set downsampling can be even more > 100
  //motor.useMonitoring(Serial, 100);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

   // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the target velocity [rad/s] 
  motor.target = 0; // one rotation per second

  //command.add('M', doMotor, "Motor");
  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  bool currentButtonState = digitalRead(SAFETY_PIN);
  if (sv.lastButtonState == HIGH && currentButtonState == LOW) {
    // toggle motor state
    sv.motorOn = !sv.motorOn;
  }
  sv.lastButtonState = currentButtonState;
  
  // main FOC algorithm function
  

  // encoder.update();
  // Serial.print(encoder.getAngle());
  // Serial.print("\t");
  // Serial.println(encoder.getVelocity());
  
  if (sv.motorOn == 1) {
    motor.loopFOC();
    motor.move();
  }

  // display the currents
  //motor.monitor();

  // user communication
  //command.run();
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