void interpret_ir_key(char key){
  if (key){
    // Serial.println(key);

    if(key == 'f' || key =='r') {
      input_values = ""; // clear input password
    } else if(key == 's') {
      new_damping_values = input_values; // convert input password to integer
      input_values = ""; // clear input password
      set_front_damping(new_damping_values.substring(0,2).toInt());
      set_rear_damping(new_damping_values.substring(2,4).toInt());
      digitalWrite(BUZZER_PIN, HIGH);
      delay(10);
      digitalWrite(BUZZER_PIN, LOW);
    } else if(key == 'p') {
      // stop motors
      FL_stepper.disableOutputs();
      FR_stepper.disableOutputs();
      RL_stepper.disableOutputs();
      RR_stepper.disableOutputs();
      new_damping_values = "-999";
    } else if(key == 'D') {
      // front decrease
      decrease_front_damping(1);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'I') {
      // front increase
      increase_front_damping(1);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'd') {
      // rear decrease
      decrease_rear_damping(1);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'i') {
      // rear increase
      increase_rear_damping(1);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'h') {
      // set home
      new_damping_values = "0000";
      damping_update_flag = false;
      decrease_front_damping(front_Damping_Value);
      decrease_rear_damping(rear_Damping_Value);
      FL_stepper.setCurrentPosition(0);
      FR_stepper.setCurrentPosition(0);
      RL_stepper.setCurrentPosition(0);
      RR_stepper.setCurrentPosition(0);
      FL_stepper.enableOutputs();
      FR_stepper.enableOutputs();
      RL_stepper.enableOutputs();
      RR_stepper.enableOutputs();
    } else {
      if (input_values.length() < 4) {
        input_values += key; // append new character to input password string
      }
    }
  }
}


void interpret_pad_key(char key){
  if (key){
    // Serial.println(key);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    if(key == '#'){
      if (input_values.length() == 4) {
        new_damping_values = input_values; // convert input password to integer
        input_values = ""; // clear input password
        set_front_damping(new_damping_values.substring(0,2).toInt());
        set_rear_damping(new_damping_values.substring(2,4).toInt());
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
      }
    } else if(key == '*') {
      if (input_values.length() > 0) {
        input_values = ""; // clear input password
      }else if (new_damping_values == "-999") {
        // set home
        new_damping_values = "0000";
        damping_update_flag = false;
        decrease_front_damping(front_Damping_Value);
        decrease_rear_damping(rear_Damping_Value);
        FL_stepper.setCurrentPosition(0);
        FR_stepper.setCurrentPosition(0);
        RL_stepper.setCurrentPosition(0);
        RR_stepper.setCurrentPosition(0);
        FL_stepper.enableOutputs();
        FR_stepper.enableOutputs();
        RL_stepper.enableOutputs();
        RR_stepper.enableOutputs();
      } else {
        // stop motors
        FL_stepper.disableOutputs();
        FR_stepper.disableOutputs();
        RL_stepper.disableOutputs();
        RR_stepper.disableOutputs();
        new_damping_values = "-999";
      }
    } else if(key == 'B') {
      // front decrease
      decrease_front_damping(5);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'A') {
      // front increase
      increase_front_damping(5);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'D') {
      // rear decrease
      decrease_rear_damping(5);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else if(key == 'C') {
      // rear increase
      increase_rear_damping(5);
      new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
    } else {
      if (input_values.length() < 4) {
        input_values += key; // append new character to input password string
      } else if (input_values.length() == 4) {
        input_values = ""; // clear input password
        input_values += key; // append new character to input password string
      }
    }
  }
}