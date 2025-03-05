#include "simulation.h"
#include "math.h"

void Simulation::newState() {
  if (current != previous) {
    newStateStartTime = _micros();
    previous = current;

    // random noise wait time
    if (current == SimulationState::RANDOM_NOISE) {
      random_noise_wait = random(10, 26) * 1000000UL;
      Serial.print("Random noise wait time: ");
      Serial.println(random_noise_wait);
      perLoopStartTime = newStateStartTime;
    }

    begining_motor_0_position = motor_0->shaftAngle();
    //begining_motor_1_position = motor_1->shaftAngle();
    simulation_state_stablized = false;
  }
}    

bool Simulation::motionDetectedForMotor(ComponentID motor_id, float previousPos = 0.0f) {
    float begining_position = previousPos == 0.0f ? motor_id == ComponentID::ZERO ? begining_motor_0_position : begining_motor_1_position: previousPos;
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    if (abs(current_position - begining_position) > movement_threshold) {
        return true;
    }
    return false;
}

void Simulation::setMotorPosition(ComponentID motor_id, float position) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    cur_motor->target = position;
}

bool Simulation::atLocationForMotor(ComponentID motor_id, float position) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    if (abs(current_position - position) < stablized_threshold) {
        return true;
    }
    return false;
}

void Simulation::moveMotor(ComponentID motor_id, float targetPosition, float targetVelocity, float targetVoltage) {
    BLDCMotor* cur_motor = getMotor(motor_id);
    cur_motor->voltage_limit = targetVoltage;
    cur_motor->velocity_limit = targetVelocity;
    cur_motor->move(targetPosition);
}

ComponentID Simulation::selectRandomMotor() {
    return random(0, 2) == 0 ? ComponentID::ZERO : ComponentID::ONE;
}

void Simulation::updateIDLE() {
    /*
    if (motionDetectedForMotor(ComponentID::ZERO) || motionDetectedForMotor(ComponentID::ONE)) {
        current = SimulationState::CALIBRATION;
    }*/
   if (motionDetectedForMotor(ComponentID::ZERO)) {
        current = SimulationState::CALIBRATION;
    } 
}

void Simulation::updateCALIBRATION() {
    motor_0->controller = MotionControlType::velocity;
    motor_0->move(3);
    // calibrated_position = 0.0;
    // setMotorPosition(ComponentID::ZERO, calibrated_position);
    // //setMotorPosition(ComponentID::ONE, calibrated_position);
    // moveMotor(ComponentID::ZERO, calibrated_position, calibration_velocity, calibration_voltage);
    // //moveMotor(ComponentID::ONE, calibrated_position, calibration_velocity, calibration_voltage);
    // //if (!atLocationForMotor(ComponentID::ZERO, calibrated_position) || !atLocationForMotor(ComponentID::ONE, calibrated_position)) {
    // if (!atLocationForMotor(ComponentID::ZERO, calibrated_position)) {
    //     // do nothing and wait
    // } else if (!simulation_state_stablized){
    //     simulation_state_stablized = true;
    //     newStateStartTime = _micros();
    if (motionDetectedForMotor(ComponentID::ZERO, motor_0->shaft_angle)) { //test this should keep resetting time until motor stays within threshold 
        newStateStartTime = _micros();
    } else if ((_micros() - newStateStartTime) > calibration_wait) {
        motor_0->move(0);
        motor_0->controller = MotionControlType::angle;
        Serial.println("Calibration for motor 0 stabled at position: ");
        calibrated_position = motor_0->shaftAngle();
        Serial.println(motor_0->shaftAngle());
        //Serial.println("Calibration for motor 1 stabled at position: ");
        //Serial.println(motor_1->shaftAngle());
        current = SimulationState::RANDOM_NOISE;
    }
}

void Simulation::updateRANDOM_NOISE() {
    unsigned long current_time = _micros();
    // motor_0->controller = MotionControlType::velocity_openloop;
    // float vel = 0.0;
    if ((current_time - newStateStartTime) > random_noise_wait) {
        current = SimulationState::INCIDENT;
    } else if((current_time - perLoopStartTime) > random_noise_interval) {
        //do nothing for now

        perLoopStartTime = current_time;
        // // float new_target = calibrated_position + random_noise_threshold * last_random_noise_direction;
        // // Serial.println("Random noise new target position per second: ");
        // // Serial.println(new_target);
        // // last_random_noise_direction = -last_random_noise_direction;
        // // moveMotor(ComponentID::ZERO, new_target, random_noise_velocity, random_noise_voltage);
        // //moveMotor(ComponentID::ONE, new_target, random_noise_velocity, random_noise_voltage);
        // motor_0->voltage_limit = 11;
        // vel = 2 * last_random_noise_direction;
        // last_random_noise_direction = -last_random_noise_direction;
        // Serial.println(vel);
    }
    //motor_0->move(vel);
}
//incident should be good, just test
void Simulation::updateINCIDENT() { 
    // incident_motor = selectRandomMotor();
    moveMotor(incident_motor, incident_position, incident_velocity, incident_voltage);
    if (!atLocationForMotor(incident_motor, incident_position)) {
        // do nothing and wait
    } else if (!simulation_state_stablized){
        simulation_state_stablized = true;
        Serial.println("Incident motor has arrived at the incident position: ");
        Serial.println(getMotor(incident_motor)->shaftAngle());
        current = SimulationState::RESPONSE;
    }
}

void Simulation::updateRESPONSE() {
    ComponentID react_motor = incident_motor == ComponentID::ZERO ? ComponentID::ONE : ComponentID::ZERO;
    BLDCMotor* cur_motor = getMotor(react_motor);
    unsigned long current_time = _micros();
    if ((current_time - newStateStartTime) <= response_time_wait) {

        if (motionDetectedForMotor(react_motor, cur_motor->shaft_angle)) { 
            // there is user movement; for simplicity, we don't care how far the movement is
            Serial.println("Passed!");
            current = SimulationState::IDLE;
        } // no action taken otherwise
    } else { // no action within 3 seconds after motor arrival
        // TODO - implement failure by buzzing the motor or something
        Serial.println("failed!");
        current = SimulationState::CALIBRATION;
    }
}
