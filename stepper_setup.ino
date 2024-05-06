void setup_speedy_stepper(SpeedyStepper stepper){
  stepper.setStepsPerRevolution(200*MICROSTEPS);
  stepper.setSpeedInRevolutionsPerSecond(SPEED);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(4);
  // stepper.setCurrentPositionInRevolutions(0);
}