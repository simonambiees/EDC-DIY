char ir_get_key() {
  char key = 0;
  if (IrReceiver.decode()) {
    uint16_t command = IrReceiver.decodedIRData.command;
    switch (command) {
      case 68:
        Serial.println("Front Decrease");
        key = 'D';
        break;
      case 67:
        Serial.println("Front Increase");
        key = 'I';
        break;
      case 7:
        Serial.println("Rear Decrease");
        key = 'd';
        break;
      case 9:
        Serial.println("Rear Increase");
        key = 'i';
        break;
      case 64:
        Serial.println("Select Front");
        key = 'f';
        break;
      case 21:
        Serial.println("Select Rear");
        key = 'r';
        break;
      case 69:
        Serial.println("Stop Motors");
        key = 'p';
        break;
      case 71:
        Serial.println("Set Home");
        key = 'h';
        break;
      case 13:
        Serial.println("Set Values");
        key =  's';
        break;
      case 22:
        Serial.println("0");
        key = '0';
        break;
      case 12:
        Serial.println("1");
        key = '1';
        break;
      case 24:
        Serial.println("2");
        key = '2';
        break;
      case 94:
        Serial.println("3");
        key = '3';
        break;
      case 8:
        Serial.println("4");
        key = '4';
        break;
      case 28:
        Serial.println("5");
        key = '5';
        break;
      case 90:
        Serial.println("6");
        key = '6';
        break;
      case 66:
        Serial.println("7");
        key = '7';
        break;
      case 82:
        Serial.println("8");
        key = '8';
        break;
      case 74:
        Serial.println("9");
        key = '9';
        break;
      default:
        Serial.println("UNDEFINED");
        key = 0;
    }
    delay(500);
    IrReceiver.resume();
  }
  return key;
}