#include <EEPROM.h>

#define FRONT_DAMPING_EEPROM_ADDR 0
#define REAR_DAMPING_EEPROM_ADDR  1

void init_eeprom() {
  EEPROM.write(FRONT_DAMPING_EEPROM_ADDR, 0);
  EEPROM.write(REAR_DAMPING_EEPROM_ADDR, 0);
}

void home_damping_knobs(){
  FL_stepper.setupRelativeMoveInRevolutions(HOMING_REVS);
  FR_stepper.setupRelativeMoveInRevolutions(HOMING_REVS);
  RL_stepper.setupRelativeMoveInRevolutions(HOMING_REVS);
  RR_stepper.setupRelativeMoveInRevolutions(HOMING_REVS);

  // Homing FL
  while((!FL_Stall_State) && (!FL_stepper.motionComplete()))
  {
    FL_stepper.processMovement();
  }
  if(!FL_Stall_State)
  {
    // Serial.println("FL Homing Error.");
    Serial.println("lalala.");
  }
  else
  {
    Serial.println("FL Homing Success.");
    FL_stepper.setupStop();
    FL_stepper.setCurrentPositionInSteps(0);
    FL_Stall_State = false;
  }

  // Homing FR
  while((!FR_Stall_State) && (!FR_stepper.motionComplete()))
  {
    FR_stepper.processMovement();
  }
  if(!FR_Stall_State)
  {
    Serial.println("FR Homing Error.");
  }
  else
  {
    Serial.println("FR Homing Success.");
    FR_stepper.setupStop();
    FR_stepper.setCurrentPositionInSteps(0);
    FR_Stall_State = false;
  }

  // Homing RL
  while((!RL_Stall_State) && (!RL_stepper.motionComplete()))
  {
    RL_stepper.processMovement();
  }
  if(!RL_Stall_State)
  {
    Serial.println("RL Homing Error.");
  }
  else
  {
    Serial.println("RL Homing Success.");
    RL_stepper.setupStop();
    RL_stepper.setCurrentPositionInSteps(0);
    RL_Stall_State = false;
  }

  // Homing RR
  while((!RR_Stall_State) && (!RR_stepper.motionComplete()))
  {
    RR_stepper.processMovement();
  }
  if(!RR_Stall_State)
  {
    Serial.println("RR Homing Error.");
  }
  else
  {
    Serial.println("RR Homing Success.");
    RR_stepper.setupStop();
    RR_stepper.setCurrentPositionInSteps(0);
    RR_Stall_State = false;
  }
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
  if ((front_Damping_Value + steps) > DAMPING_STEP_COUNT) {
    steps = DAMPING_STEP_COUNT - front_Damping_Value;
  }
  front_Damping_Value += steps;
  if (front_Damping_Value > DAMPING_STEP_COUNT) {
    front_Damping_Value = DAMPING_STEP_COUNT;
  }
  if (front_Damping_Value < 0) {
    front_Damping_Value = 0;
  }
  save_damping_values();
}

void decrease_front_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((front_Damping_Value - steps) < 0) {
    steps = front_Damping_Value - 0;
  }
  front_Damping_Value -= steps;
  if (front_Damping_Value > DAMPING_STEP_COUNT) {
    front_Damping_Value = DAMPING_STEP_COUNT;
  }
  if (front_Damping_Value < 0) {
    front_Damping_Value = 0;
  }
  save_damping_values();
}

void increase_rear_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((rear_Damping_Value + steps) > DAMPING_STEP_COUNT) {
    steps = DAMPING_STEP_COUNT - rear_Damping_Value;
  }
  rear_Damping_Value += steps;
  if (rear_Damping_Value > DAMPING_STEP_COUNT) {
    rear_Damping_Value = DAMPING_STEP_COUNT;
  }
  if (rear_Damping_Value < 0) {
    rear_Damping_Value = 0;
  }
  save_damping_values();
}

void decrease_rear_damping(int steps) {
  if (steps < 0) {
    return;
  }
  if ((rear_Damping_Value - steps) < 0) {
    steps = rear_Damping_Value;
  }
  rear_Damping_Value -= steps;
  if (rear_Damping_Value > DAMPING_STEP_COUNT) {
    rear_Damping_Value = DAMPING_STEP_COUNT;
  }
  if (rear_Damping_Value < 0) {
    rear_Damping_Value = 0;
  }
  save_damping_values();
}


void update_damping_knobs(){
  
  // digitalWrite(EN_PIN, LOW);
  FL_stepper.setupMoveInRevolutions(front_Damping_Value*REV_PER_DAMPING_STEP);
  FR_stepper.setupMoveInRevolutions(front_Damping_Value*REV_PER_DAMPING_STEP);
  RL_stepper.setupMoveInRevolutions(rear_Damping_Value*REV_PER_DAMPING_STEP);
  RR_stepper.setupMoveInRevolutions(rear_Damping_Value*REV_PER_DAMPING_STEP);

  // Setting FL
  while((!FL_Stall_State) && (!FL_stepper.motionComplete()))
  {
    FL_stepper.processMovement();
  }

  // Setting FR
  while((!FR_Stall_State) && (!FR_stepper.motionComplete()))
  {
    FR_stepper.processMovement();
  }

  // Setting RL
  while((!RL_Stall_State) && (!RL_stepper.motionComplete()))
  {
    RL_stepper.processMovement();
  }

  // Setting RR
  while((!RR_Stall_State) && (!RR_stepper.motionComplete()))
  {
    RR_stepper.processMovement();
  }
  // digitalWrite(EN_PIN, HIGH);

}