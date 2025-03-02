void blinkyLight(void);

// interrupt routine initialisation
void doA(void);
void doB(void);
void doIndex(void);

//store state here
struct SensorState {
  bool motorOn;
  bool lastButtonState;
};

void updateStuff(void);