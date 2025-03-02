enum struct MountSide {
  LEFT,
  RIGHT
};

enum struct MotorID {
  MOTOR_ZERO = 0,
  MOTOR_ONE
};

//state machine
enum struct Simulation {
  IDLE,
  CALIBRATION,
  RANDOM_NOISE,
  INCIDENT,
  RESPONSE,
  INVALID
};

struct SimulationState {
  Simulation current;
  Simulation previous;
  unsigned int random_noise_rand;

  SimulationState() {
    current = Simulation::IDLE;
    previous = Simulation::INVALID;
    random_noise_rand = -1;
  }
};
/*
struct SensorState {
};*/

// interrupt routine initialisation
void doA(void);
void doB(void);
// void doIndex(void);

/*
  targetPosition
    - reel down -- - values in meters from starting position
    - reel up  -- + values in meters from the starting position
  targetVelocity - in meters/second (d/t)
  motor - motor 1 or 2
  side - which side is the motor mounted on. CW or CCW conversion for out and in values.
*/
void updatePositionAndSpeed(float targetPosition, float targetVelocity, MotorID motor, MountSide side);

//sits in the update loop and adjusts motor
void move(float targetPosition, float currentPosition, float targetVelocity, float currentVelocity, MotorID motor, MountSide side);

//random functions 
unsigned int randomRange(int start, int end);
MotorID selectMotor(void);

void updateStuff(void);


