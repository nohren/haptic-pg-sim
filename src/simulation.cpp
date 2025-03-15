#include "simulation.h"
#include "math.h"

//easier debugging
template <typename T>
void print(const T& value) {
  Serial.print(value);
}
template <typename T>
void println(const T& value) {
    Serial.println(value);
}

bool Simulation::motionDetectedForMotor(ComponentID motor_id) {
    float previous_position = motor_id == ComponentID::ZERO ? previous_position_0 : previous_position_1;
    BLDCMotor* cur_motor = getMotor(motor_id);
    float current_position = cur_motor->shaftAngle();
    if (abs(current_position - previous_position) > movement_threshold) {
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
  
      simulation_state_stablized = false;
    }
}    

void Simulation::updateIDLE() {
    /*
    if (motionDetectedForMotor(ComponentID::ZERO) || motionDetectedForMotor(ComponentID::ONE)) {
        current = SimulationState::CALIBRATION;
    }*/
    if (motionDetectedForMotor(ComponentID::ZERO) && ((_micros() - newStateStartTime) > idle_interval)) {
        current = SimulationState::CALIBRATION;
    } 
}

void Simulation::updateCALIBRATION() {
    if (!simulation_state_stablized) {
        motor_0->controller = MotionControlType::velocity;
        move = -3;
    }
    
    if (motionDetectedForMotor(ComponentID::ZERO) && !simulation_state_stablized) { 
        newStateStartTime = _micros();
       
    } else if ((_micros() - newStateStartTime) > calibration_wait) {
        if (!simulation_state_stablized) {
            Serial.println("Calibration for motor 0 stable at position: ");
            motor_0->controller = MotionControlType::angle;
            move = motor_0->shaftAngle();
        }
        simulation_state_stablized = true;
        current = SimulationState::RANDOM_NOISE;
    }
}

void Simulation::updateRANDOM_NOISE() {
    unsigned long current_time = _micros();
    if (!simulation_state_stablized) {
        motor_0->controller = MotionControlType::velocity;
        move = -1;
    }
    // float vel = 0.0;
    if ((current_time - newStateStartTime) > random_noise_wait) {
        current = SimulationState::INCIDENT;
    } else if((current_time - perLoopStartTime) > random_noise_interval) {
        //do nothing for now

        perLoopStartTime = current_time;
        // float new_target = calibrated_position + random_noise_threshold * last_random_noise_direction;
        // Serial.println("Random noise new target position per second: ");
        // Serial.println(new_target);
        // last_random_noise_direction = -last_random_noise_direction;
        // moveMotor(ComponentID::ZERO, new_target, random_noise_velocity, random_noise_voltage);
        //moveMotor(ComponentID::ONE, new_target, random_noise_velocity, random_noise_voltage);

        vel = 2 * last_random_noise_direction;
        last_random_noise_direction = -last_random_noise_direction;
        Serial.println(vel);
    }
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

        if (motionDetectedForMotor(react_motor)) { 
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
