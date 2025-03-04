#ifndef SIMULATION_h
#define SIMULATION_h

#include <SimpleFOC.h>
#include "math.h"

/*
IDLE---->CALIBRATION---->RANDOM_NOISE---->INCIDENT---->RESPONSE
             ^                                            |
             |____________________________________________|

*/
enum class SimulationState : uint8_t {
  IDLE = 0,
  CALIBRATION,
  RANDOM_NOISE,
  INCIDENT,
  RESPONSE,
  INVALID
};

inline String SimulationStateToString(SimulationState s) {
    switch (s) {
      case SimulationState::IDLE:   return "IDLE";
      case SimulationState::CALIBRATION: return "CALIBRATION";
      case SimulationState::RANDOM_NOISE:  return "RANDOM_NOISE";
      case SimulationState::INCIDENT: return "INCIDENT";
      case SimulationState::RESPONSE: return "RESPONSE";
      default:    return "UNKNOWN";
    }
}
//------------------------------MOTOR----------------------------

enum class ComponentID {
  ZERO, 
  ONE 
};

class Simulation {
  public:
    Encoder* encoder_0 = nullptr;
    Encoder* encoder_1 = nullptr;

    BLDCDriver* driver_0 = nullptr;
    BLDCDriver* driver_1 = nullptr;

    BLDCMotor* motor_0 = nullptr;
    BLDCMotor* motor_1 = nullptr;

    SimulationState current = SimulationState::IDLE;
    SimulationState previous = SimulationState::INVALID;

    float begining_motor_0_position;
    float begining_motor_1_position;

    void init(Encoder* cur_encoder_0, Encoder* cur_encoder_1, BLDCDriver* cur_driver_0, BLDCDriver* cur_driver_1, BLDCMotor* cur_motor_0, BLDCMotor* cur_motor_1) {
        encoder_0 = cur_encoder_0;
        encoder_1 = cur_encoder_1;
        driver_0 = cur_driver_0;
        driver_1 = cur_driver_1;
        motor_0 = cur_motor_0;
        motor_1 = cur_motor_1;
    }

    void initSingle(Encoder* cur_encoder_0, BLDCDriver* cur_driver_0, BLDCMotor* cur_motor_0) {
        encoder_0 = cur_encoder_0;
        driver_0 = cur_driver_0;
        motor_0 = cur_motor_0;
    }

    // Update related variables after entering a new state
    void newState();

    // Resting state for all components and waiting for motion
    void updateIDLE(); 

    // Set target position on motors 
    void updateCALIBRATION();

    // Mimic flying
    void updateRANDOM_NOISE();

    // Drop one of the motors to a specific position
    void updateINCIDENT();

    // React to the other motor's movement
    void updateRESPONSE();

  private:
    Encoder* getEncoder(ComponentID component_id) {
      return component_id == ComponentID::ZERO ? encoder_0 : encoder_1;
    }
    BLDCDriver* getDriver(ComponentID component_id) {
      return component_id == ComponentID::ZERO ? driver_0 : driver_1;
    }
    BLDCMotor* getMotor(ComponentID component_id) {
      return component_id == ComponentID::ZERO ? motor_0 : motor_1;
    }

    // ------------motor movement --------------
    bool motionDetectedForMotor(ComponentID motor_id);

    void setMotorPosition(ComponentID motor_id, float position);

    bool atLocationForMotor(ComponentID motor_id, float position);

    void moveMotor(ComponentID motor_id, float targetPosition, float targetVelocity, float targetVoltage);

    ComponentID selectRandomMotor();

    // ------------tracker --------------
    unsigned long newStateStartTime = 0; // track time we entered a new current state
    unsigned long perLoopStartTime = 0; // track time we enter a new loop
    bool simulation_state_stablized = false;

    // ----------- parameters ------------ 
    float calibrated_position = 0.0;
    float incident_position = 10.0;

    unsigned long calibration_wait = 5000000UL;
    unsigned long response_time_wait = 3000000UL;
    unsigned long random_noise_wait = -1UL;
    unsigned long random_noise_interval = 2000000UL;

    // more than half a rotation is considered an intentional movement
    float movement_threshold = 0.5 * _PI; // 1/4 rotation

    // within 1/20 rotation is considered stable position
    float stablized_threshold = 0.1 * _PI; // 1/20 rotation

    // move back and forth within 1/10 rotation
    float random_noise_threshold = 0.2 * _PI; // 1/10 rotation
    int last_random_noise_direction = 1;

    float calibration_voltage = 11.0;
    float calibration_velocity = 4.0;
    float random_noise_voltage = 5.0;
    float random_noise_velocity = 10.0;
    float incident_voltage = 10.0;
    float incident_velocity = 10.0;
    float response_voltage = 5.0;
    float response_velocity = 10.0;

    ComponentID incident_motor = ComponentID::ZERO;
};

#endif
