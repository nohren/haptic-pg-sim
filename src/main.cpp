#include "simulation.h"
#include "components_helper.h"

//------------------------------ENCODER--------------------------
// Encoder(int encA, int encB , float ppr, int index = 0);
Encoder encoder_0 = Encoder(2, 3, 2048);
Encoder encoder_1 = Encoder(2, 3, 2048);

//------------------------------DIRVER---------------------------
// BLDCDriver3PWM(int pinA, int pinB, int pinC, int enable);
// 8 is enable, these pin numbers are on the back of the motor driver corresponding to a,b,c
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(5, 9, 6, 8);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(5, 9, 6, 8);

//uses up a lot of memory and not working for the params.  Too much to learn for too short a time!
//InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 20, A0, A1, A2);

// motor info!
// GM4108H
// https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217?srsltid=AfmBOoqxeW2Hb-j6NUhU7VB4WXt1Kw264wGX11YsdpgPQk_aYoP7Nqlo
// 22 poles, 11 poll pairs
// Resistance(Ω): 11.1Ω±5%
// Load Current (A): 1.5A
// diameter 47mm or 0.047m
// radius 0.0235
// circumference 0.1476 or 2pi radians
// 6.77 full rotations or 42.56 radians
// BLDCMotor(int pole_pairs);
BLDCMotor motor_0 = BLDCMotor(11);
BLDCMotor motor_1 = BLDCMotor(11);


//-------------------------SIMULATION-------------------------------
Simulation simulation = Simulation();

//-------------------------COMMAND-------------------------------
//Commander command = Commander(Serial); //remove to save memory
//void doMotor(char* cmd) { command.motor(&motor, cmd); }
//void doAngle(char* cmd) { command.scalar(&simulation.targetAngle, cmd); }

void initDriver(BLDCDriver3PWM* driver) {
  driver->voltage_power_supply = 11.1;
  driver->voltage_limit = 11.1; //amps = voltage / ohms  11.1v battery / 11 ohms = 1amp < 1.5 amp ... we good
  if(!driver->init()) {
    Serial.println("Driver init failed!");
    return;
  }
}

void initMotor(BLDCMotor* motor, Encoder* encoder, BLDCDriver3PWM* driver) {
  /*
  Update motor parameters
  */
 // Handle and initialize motor
  motor->torque_controller = TorqueControlType::voltage; 
  motor->controller = MotionControlType::angle;
  
  //PID default 0.05,10,0.001
  motor->PID_velocity.P = 0.1; //proportional gain - used to correct current and target angle/velocity. higher is more aggressive.
  motor->PID_velocity.I = 10; //for correcting velocity error
  motor->PID_velocity.D = 0.001; //damps sudden changes to angle velocity
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor->PID_velocity.output_ramp = 1000;

  motor->P_angle.P = 5;   // angle P controller -  default P=20
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor->LPF_velocity.Tf = 0.01;
  // set to target velocity limit
  motor->velocity_limit = 0.0; //maximal velocity of the position control
  //key for reducing torque!
  motor->voltage_limit = 4;

  motor->linkSensor(encoder);
  motor->linkDriver(driver);

  if(!motor->init()){
    Serial.println("Motor init failed!");
    return;
  }
  if(!motor->initFOC()){
    Serial.println("FOC init failed!");
    return;
  }
}

void setup() {
  randomSeed(analogRead(A0));
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Handle and initialize encoder
  encoder_0.init();
  encoder_0.enableInterrupts(doA_0, doB_0);
  encoder_1.init();
  encoder_1.enableInterrupts(doA_1, doB_1);


  // Handle and initialize driver
  initDriver(&driver_0);
  initDriver(&driver_1);
  
  // Handle and initialize motor
  initMotor(&motor_0, &encoder_0, &driver_0);
  initMotor(&motor_1, &encoder_1, &driver_1);
 
  // initialize the simulation
  simulation.init(&encoder_0, &encoder_1, &driver_0, &driver_1, &motor_0, &motor_1);

  // add commands to the commander
  //command.add('M', doMotor, "Motor");
  //command.add('A', doAngle, "angle");
  Serial.println(F("Ready!"));
  _delay(1000);
}

void loop() {
  motor_0.loopFOC();
  motor_1.loopFOC();
  /*
  if (simulation.current != simulation.previous) {
    simulation.newState();
  }

  switch (simulation.current){
    case SimulationState::IDLE:
      Serial.println("IDLE State");
      simulation.updateIDLE();
      break;
    case SimulationState::CALIBRATION:
      Serial.println("CALIBRATION State");
      simulation.updateCALIBRATION();
      break;
    case SimulationState::RANDOM_NOISE:
      Serial.println("RANDOM_NOISE State");
      simulation.updateRANDOM_NOISE();
      break;
    case SimulationState::INCIDENT:
      Serial.println("INCIDENT State");
      simulation.updateINCIDENT();
      break;
    case SimulationState::RESPONSE:
      Serial.println("RESPONSE State");
      simulation.updateRESPONSE();
      break;
    case SimulationState::RESET:
      Serial.println("RESET State");
      simulation.updateRESET();
      break;  
    default:
      Serial.println("INVALID State");
      // weird simulation state detected
      break;
    }

  */
 motor_0.move(20.0);
}

//------------------------------ENCODER HELPER--------------------------
void doA_0(){encoder_0.handleA();}
void doB_0(){encoder_0.handleB();}
void doA_1(){encoder_1.handleA();}
void doB_1(){encoder_1.handleB();}
// void doIndex_0(){encoder_0.handleIndex();}
// void doIndex_1(){encoder_0.handleIndex();}