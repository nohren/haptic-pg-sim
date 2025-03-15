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
    unsigned long currentTime = _micros();
    if (!simulation_state_stablized) {
        motor_0->controller = MotionControlType::velocity;
        move = -3;
    }
    
    if (motionDetectedForMotor(ComponentID::ZERO) && !simulation_state_stablized) { 
        newStateStartTime = currentTime;
       
    } else if ((currentTime - newStateStartTime) > calibration_wait) {
        if (!simulation_state_stablized) {
            Serial.println("Calibration for motor 0 stable at position: ");
            motor_0->controller = MotionControlType::angle;
            calibrated_position = motor_0->shaftAngle();
            move = calibrated_position;
            simulation_state_stablized = true;
        }

        //little bit of wait before we get into some normal flying turbulence
        if ((currentTime - newStateStartTime) > (calibration_wait * 5UL)) {
            current = SimulationState::RANDOM_NOISE;
        }  
    }
}

void Simulation::updateRANDOM_NOISE() {
    unsigned long current_time = _micros();
    float moveFactor = 2.0;
    if (!simulation_state_stablized) {
        motor_0->controller = MotionControlType::velocity;
        move = 0;
    }
    if ((current_time - perLoopStartTime) > random_noise_interval) {
        simulation_state_stablized = true;
        perLoopStartTime = current_time;
        move = last_random_noise_direction * moveFactor;
        last_random_noise_direction = -last_random_noise_direction;
    }
    //sort of like a do-while loop
    if ((current_time - newStateStartTime) > random_noise_wait) {
        motor_0->controller = MotionControlType::angle;
        current = SimulationState::INCIDENT;
    }
}

void Simulation::updateINCIDENT() { 
    // incident_motor = selectRandomMotor();
    float incidentPos = calibrated_position + 15.0f;
    if (!simulation_state_stablized) {
       move = incidentPos;
    }
    //moveMotor(incident_motor, incident_position, incident_velocity, incident_voltage);
    if (atLocationForMotor(incident_motor, incidentPos)) {
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
