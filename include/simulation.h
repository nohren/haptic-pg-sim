#ifndef SIMULATION_h
#define SIMULATION_h

#include <SimpleFOC.h>
#include "math.h"

/*
IDLE---->CALIBRATION---->RESET---->RANDOM_NOISE---->INCIDENT---->RESPONSE
                           ^             |                           |
                           |_____________|___________________________|

*/
enum class SimulationState : uint8_t {
  IDLE = 0,
  CALIBRATION,
  RESET,
  RANDOM_NOISE,
  INCIDENT,
  RESPONSE,
  INVALID
};

//------------------------------MOTOR----------------------------

enum class MotorState {
  MOVING,
  STOPPED,
  DONT_CARE
};

enum class ComponentID {
  ZERO, // LEFT MOTOR
  ONE // RIGHT MOTOR
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


    void updateIDLE(); 
    void updateCALIBRATION();
    void updateRESET();
    void updateRANDOM_NOISE();
    void updateINCIDENT();
    void updateRESPONSE();
    void newState();

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

    void SetMotorPosition(ComponentID motor_id, float position);

    bool AtLocationForMotor(ComponentID motor_id, float position);

/*
  targetPosition
    - reel down -- - values in meters from starting position
    - reel up  -- + values in meters from the starting position
  targetVelocity - in meters/second (d/t)
  motor - motor 1 or 2
  side - which side is the motor mounted on. CW or CCW conversion for out and in values.
*/
    void moveBrake(float targetPosition, float targetVelocity, ComponentID motor);

    // ------------tracker --------------
    unsigned long newStateStartTime = 0; // track time we entered a new current state
    unsigned long perLoopStartTime = 0; // track time we enter a new loop
    bool simulation_state_stablized = false;

    // ----------- parameters ------------ 
    float calibrated_position = 0.0;
    unsigned long random_noise_rand = -1UL;
    
    unsigned long reset_state_wait = 5000000UL;
    unsigned long response_time_wait = 3000000UL;
    float movementThreshold = _PI; // half a rotation

    //float currentPosition_0 = 0.0;
    float targetPosition_0 = 0.0;
    //float currentVelocity_0 = 0.0;
    float targetVelocity_0 = 0.0;
    //float currentPosition_1 = 0.0;
    float targetPosition_1 = 0.0;
    //float currentVelocity_1 = 0.0;
    float targetVelocity_1 = 0.0;

    //motor movement 
    float targetAngle = 0.0;
    float targetVoltage = 0.0;
};

#endif