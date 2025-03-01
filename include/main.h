enum struct MountSide {
  LEFT,
  RIGHT
};

enum struct MotorID {
  MOTOR_ONE = 0,
  MOTOR_TWO
};

//state machine
enum struct Simulation {
  CALIBRATION,
  RANDOM_NOISE,
  DROP_MOTOR,
  PAUSE_AFTER_DROP,
  READ_MOTOR,
  IDLE
};

struct SensorState {
  bool motorOn;
  bool lastButtonState;
};

// interrupt routine initialisation
void doA(void);
void doB(void);
// void doIndex(void);

/*
  distance
    - reel out -- are + values in meters from current position
    - reel in  -- are - values in meters from the current position
  velocity - in meters/second (d/t)
  motor - which motor 1 or 2?
  side - which side is the motor mounted on. CW or CCW conversion for out and in values.
*/
void move(float distance, float velocity, MotorID motor, MountSide side);

//random functions 
int randomRange(int start, int end);
MotorID selectMotor(void);


