#define DOCTEST_CONFIG_IMPLEMENT  // REQUIRED: Enable custom main()
#include <doctest.h>
// #include "simulation.h"

float begining_motor_0_position = 0;
float begining_motor_1_position = 0;
bool motionDetectedForMotor(int motor_id, float currentPos, float threshold, float previousPos = 0.0f) {
  float begining_position = previousPos == 0.0f ? motor_id == 0 ? begining_motor_0_position : begining_motor_1_position: previousPos;
  // BLDCMotor* cur_motor = getMotor(motor_id);
  float current_position = currentPos;
  if (abs(current_position - begining_position) > threshold) {
      return true;
  }
  return false;
}
// TEST_CASE ...
// TEST_SUITE ...
TEST_CASE("Sample test") {
  CHECK(motionDetectedForMotor(0, 5, 1) == true);
  CHECK(motionDetectedForMotor(0, 0.5, 1) == false);
  CHECK(motionDetectedForMotor(0, 0.5, 1, 5) == true);
}

int main(int argc, char **argv)
{
  doctest::Context context;

  // BEGIN:: PLATFORMIO REQUIRED OPTIONS
  context.setOption("success", true);     // Report successful tests
  context.setOption("no-exitcode", true); // Do not return non-zero code on failed test case
  // END:: PLATFORMIO REQUIRED OPTIONS

  // YOUR CUSTOM DOCTEST OPTIONS

  context.applyCommandLine(argc, argv);
  return context.run();
}
