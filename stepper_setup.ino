void setup_stepper_front(AccelStepper &stepper){
  stepper.setCurrentPosition(front_Damping_Value*STEPS_PER_DAMPING);
  stepper.setMaxSpeed(SPEED*STEPS_PER_REV);
  stepper.setAcceleration(2*STEPS_PER_REV);
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  steppers.addStepper(stepper);
}

void setup_stepper_rear(AccelStepper &stepper){
  stepper.setCurrentPosition(rear_Damping_Value*STEPS_PER_DAMPING);
  stepper.setMaxSpeed(SPEED*STEPS_PER_REV);
  stepper.setAcceleration(2*STEPS_PER_REV);
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  steppers.addStepper(stepper);
}