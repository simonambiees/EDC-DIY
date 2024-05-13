#include <EEPROM.h>

#define FRONT_DAMPING_EEPROM_ADDR 0
#define REAR_DAMPING_EEPROM_ADDR  1

void init_eeprom() {
  EEPROM.write(FRONT_DAMPING_EEPROM_ADDR, 0);
  EEPROM.write(REAR_DAMPING_EEPROM_ADDR, 0);
}

void home_damping_knobs(){
  // Attach the interrupt to the rising edge of the signal
  attachInterrupt(digitalPinToInterrupt(FL_Stall_Pin), FL_Stall_Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_Stall_Pin), FR_Stall_Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RL_Stall_Pin), RL_Stall_Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_Stall_Pin), RR_Stall_Interrupt, RISING);

  FL_stepper.move(HOMING_REVS*STEPS_PER_REV);
  FR_stepper.move(HOMING_REVS*STEPS_PER_REV);
  RL_stepper.move(HOMING_REVS*STEPS_PER_REV);
  RR_stepper.move(HOMING_REVS*STEPS_PER_REV);

  // Homing FL
  while((FL_stepper.distanceToGo()))
  {
    FL_stepper.run();
    if (FL_Stall_State) {
      FL_stepper.setCurrentPosition(0);
      break;
    }
  }
  if(!FL_Stall_State)
  {
    Serial.println("FL Homing Error.");
    FL_stepper.setCurrentPosition(0);
  }
  else
  {
    Serial.println("FL Homing Success.");
    FL_Stall_State = false;
  }

  // Homing FR
  while((!FR_Stall_State) && (FR_stepper.distanceToGo()))
  {
    FR_stepper.run();
  }
  if(!FR_Stall_State)
  {
    Serial.println("FR Homing Error.");
  }
  else
  {
    Serial.println("FR Homing Success.");
    FR_stepper.setCurrentPosition(0);
    FR_Stall_State = false;
  }

  // Homing RL
  while((!RL_Stall_State) && (RL_stepper.distanceToGo()))
  {
    RL_stepper.run();
  }
  if(!RL_Stall_State)
  {
    Serial.println("RL Homing Error.");
  }
  else
  {
    Serial.println("RL Homing Success.");
    RL_stepper.setCurrentPosition(0);
    RL_Stall_State = false;
  }

  // Homing RR
  while((!RR_Stall_State) && (RR_stepper.distanceToGo()))
  {
    RR_stepper.run();
  }
  if(!RR_Stall_State)
  {
    Serial.println("RR Homing Error.");
  }
  else
  {
    Serial.println("RR Homing Success.");
    RR_stepper.setCurrentPosition(0);
    RR_Stall_State = false;
  }
  delay(1000);
  // Detach the interrupt to the signal
  detachInterrupt(digitalPinToInterrupt(FL_Stall_Pin));
  detachInterrupt(digitalPinToInterrupt(FR_Stall_Pin));
  detachInterrupt(digitalPinToInterrupt(RL_Stall_Pin));
  detachInterrupt(digitalPinToInterrupt(RR_Stall_Pin));
  
  damping_update_flag = true;
}


void restore_damping_values() {
  front_Damping_Value = EEPROM.read(FRONT_DAMPING_EEPROM_ADDR);
  rear_Damping_Value = EEPROM.read(REAR_DAMPING_EEPROM_ADDR);
}


void save_damping_values() {
  EEPROM.write(FRONT_DAMPING_EEPROM_ADDR, front_Damping_Value);
  EEPROM.write(REAR_DAMPING_EEPROM_ADDR, rear_Damping_Value);
}


void increase_front_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((front_Damping_Value + steps) > DAMPING_COUNT) {
    steps = DAMPING_COUNT - front_Damping_Value;
  }
  front_Damping_Value += steps;
  if (front_Damping_Value > DAMPING_COUNT) {
    front_Damping_Value = DAMPING_COUNT;
  }
  if (front_Damping_Value < 0) {
    front_Damping_Value = 0;
  }
  save_damping_values();
  damping_update_flag = true;
}

void decrease_front_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((front_Damping_Value - steps) < 0) {
    steps = front_Damping_Value - 0;
  }
  front_Damping_Value -= steps;
  if (front_Damping_Value > DAMPING_COUNT) {
    front_Damping_Value = DAMPING_COUNT;
  }
  if (front_Damping_Value < 0) {
    front_Damping_Value = 0;
  }
  save_damping_values();
  damping_update_flag = true;
}

void set_front_damping(int value) {
  if (value < 0) {
    value = 0;
  }
  if (value > DAMPING_COUNT) {
    value = DAMPING_COUNT;
  }
  front_Damping_Value = value;
  save_damping_values();
  damping_update_flag = true;
}

void increase_rear_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((rear_Damping_Value + steps) > DAMPING_COUNT) {
    steps = DAMPING_COUNT - rear_Damping_Value;
  }
  rear_Damping_Value += steps;
  if (rear_Damping_Value > DAMPING_COUNT) {
    rear_Damping_Value = DAMPING_COUNT;
  }
  if (rear_Damping_Value < 0) {
    rear_Damping_Value = 0;
  }
  save_damping_values();
  damping_update_flag = true;
}

void decrease_rear_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((rear_Damping_Value - steps) < 0) {
    steps = rear_Damping_Value;
  }
  rear_Damping_Value -= steps;
  if (rear_Damping_Value > DAMPING_COUNT) {
    rear_Damping_Value = DAMPING_COUNT;
  }
  if (rear_Damping_Value < 0) {
    rear_Damping_Value = 0;
  }
  save_damping_values();
  damping_update_flag = true;
}

void set_rear_damping(int value) {
  if (value < 0) {
    value = 0;
  }
  if (value > DAMPING_COUNT) {
    value = DAMPING_COUNT;
  }
  rear_Damping_Value = value;
  save_damping_values();
  damping_update_flag = true;
}

void update_damping_knobs(){
  
  
  
  if (damping_update_flag == false)
  {
    return;
  }

  damping_update_flag = false;

  // FL_stepper.moveTo(front_Damping_Value*STEPS_PER_DAMPING);
  // FR_stepper.moveTo(front_Damping_Value*STEPS_PER_DAMPING);
  // RL_stepper.moveTo(rear_Damping_Value*STEPS_PER_DAMPING);
  // RR_stepper.moveTo(rear_Damping_Value*STEPS_PER_DAMPING);
  long positions[4];
  positions[0] = front_Damping_Value*STEPS_PER_DAMPING;
  positions[1] = front_Damping_Value*STEPS_PER_DAMPING;
  positions[2] = rear_Damping_Value*STEPS_PER_DAMPING;
  positions[3] = rear_Damping_Value*STEPS_PER_DAMPING;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  // // Setting FL
  // while((FL_stepper.distanceToGo()))
  // {
  //   FL_stepper.run();
  // }

  // // Setting FR
  // while((FR_stepper.distanceToGo()))
  // {
  //   FR_stepper.run();
  // }

  // // Setting RL
  // while((RL_stepper.distanceToGo()))
  // {
  //   RL_stepper.run();
  // }

  // // Setting RR
  // while((RR_stepper.distanceToGo()))
  // {
  //   RR_stepper.run();
  // }
  
  // delay(1000);
  // FL_stepper.disableOutputs();
  // FR_stepper.disableOutputs();
  // RL_stepper.disableOutputs();
  // RR_stepper.disableOutputs();

}