/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TM1637Display.h>
#include <IRremote.h>

#define MICROSTEPS          8 // 8, 16, 32, 64 or 256
#define SPEED               0.25 // In Revolutions per Second
#define STEPS_PER_REV       200*MICROSTEPS // 200 for 1.8 degree stepper

#define HOMING_REVS         4
#define STEPS_PER_DAMPING   STEPS_PER_REV/10
#define DAMPING_COUNT  31 // actual damping minus 1

#define STALL_VALUE         25 // [0..255]

#define EN_PIN              8 // Enable
#define FL_DIR_PIN          5 // FL Direction
#define FL_STEP_PIN         2 // FL Step
#define FL_DRIVER_ADDRESS 0b00 // FL TMC2209 Driver address
#define FR_DIR_PIN          6 // FR Direction
#define FR_STEP_PIN         3 // FR Step
#define FR_DRIVER_ADDRESS 0b01 // FR TMC2209 Driver address
#define RL_DIR_PIN          7 // RL Direction
#define RL_STEP_PIN         4 // RL Step
#define RL_DRIVER_ADDRESS 0b10 // RL TMC2209 Driver address
#define RR_DIR_PIN          A4 // RR Direction
#define RR_STEP_PIN         A5 // RR Step
#define RR_DRIVER_ADDRESS 0b11 // RR TMC2209 Driver address
#define SERIAL_PORT Serial3 // TMC2208/TMC2224 HardwareSerial port

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2209Stepper FL_driver(&SERIAL_PORT, R_SENSE, FL_DRIVER_ADDRESS);
TMC2209Stepper FR_driver(&SERIAL_PORT, R_SENSE, FR_DRIVER_ADDRESS);
TMC2209Stepper RL_driver(&SERIAL_PORT, R_SENSE, RL_DRIVER_ADDRESS);
TMC2209Stepper RR_driver(&SERIAL_PORT, R_SENSE, RR_DRIVER_ADDRESS);

// Define steppers for control
AccelStepper FL_stepper = AccelStepper(FL_stepper.DRIVER, FL_STEP_PIN, FL_DIR_PIN);
AccelStepper FR_stepper = AccelStepper(FR_stepper.DRIVER, FR_STEP_PIN, FR_DIR_PIN);
AccelStepper RL_stepper = AccelStepper(RL_stepper.DRIVER, RL_STEP_PIN, RL_DIR_PIN);
AccelStepper RR_stepper = AccelStepper(RR_stepper.DRIVER, RR_STEP_PIN, RR_DIR_PIN);

MultiStepper steppers;

using namespace TMC2208_n;

// Pin connected to the signal you want to detect
const int FL_Stall_Pin = 18;
const int FR_Stall_Pin = 19;
const int RL_Stall_Pin = 20;
const int RR_Stall_Pin = 21;

// Variable to store the state of the signal
volatile bool FL_Stall_State = false;
volatile bool FR_Stall_State = false;
volatile bool RL_Stall_State = false;
volatile bool RR_Stall_State = false;

// Variable for Front and Rear Damping Values
volatile int front_Damping_Value = 0;
volatile int rear_Damping_Value = 0;

volatile bool damping_update_flag = false;

String input_values;
String new_damping_values = "0000";

#define BUZZER_PIN A7

#define IR_RECEIVE_PIN A15
#define CLK A13
#define DIO A14

// create a display object of type TM1637Display
TM1637Display display = TM1637Display(CLK, DIO);

void setup() {
  Serial.begin(9600);         // Init serial port and set baudrate
  Serial1.begin(9600);
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  // Set the signal pin as input
  pinMode(FL_Stall_Pin, INPUT);

  

  SERIAL_PORT.begin(115200);
  setup_tmc_driver(FL_driver);
  setup_tmc_driver(FR_driver);
  setup_tmc_driver(RL_driver);
  setup_tmc_driver(RR_driver);

  restore_damping_values();
  
  setup_stepper_front(FL_stepper);
  setup_stepper_front(FR_stepper);
  setup_stepper_rear(RL_stepper);
  setup_stepper_rear(RR_stepper);

  display.clear();
  display.setBrightness(7); // set the brightness to 7 (0:dimmest, 7:brightest)

  input_values.reserve(4);
  new_damping_values.reserve(4);

  new_damping_values = String(front_Damping_Value*100+rear_Damping_Value);
  // Serial1.println(new_damping_values);
  
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); 

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(A9, OUTPUT);
  digitalWrite(A9, LOW);

  
  delay(2000);
}

void loop() {
  // Ask for user input from serial
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'i':
        Serial.println("Initialized EEPROM.");
        init_eeprom();
        restore_damping_values();
        break;
      case '0':
        Serial.println("Disabled all steppers.");
        FL_stepper.disableOutputs();
        FR_stepper.disableOutputs();
        RL_stepper.disableOutputs();
        RR_stepper.disableOutputs();
        break;
      case '1':
        // Serial.println("Homing all damping.");
        // home_damping_knobs();
        Serial.println("Resetting all damping to 0.");
        decrease_front_damping(front_Damping_Value);
        decrease_rear_damping(rear_Damping_Value);
        damping_update_flag = false;
        FL_stepper.setCurrentPosition(0);
        FR_stepper.setCurrentPosition(0);
        RL_stepper.setCurrentPosition(0);
        RR_stepper.setCurrentPosition(0);
        FL_stepper.enableOutputs();
        FR_stepper.enableOutputs();
        RL_stepper.enableOutputs();
        RR_stepper.enableOutputs();
        break;
      case 'q':
        Serial.println("Increasing front damping.");
        increase_front_damping(1);
        break;
      case 'w':
        Serial.println("Increasing rear damping.");
        increase_rear_damping(1);
        break;
      case 'a':
        Serial.println("Decreasing front damping.");
        decrease_front_damping(1);
        break;
      case 's':
        Serial.println("Decreasing rear damping.");
        decrease_rear_damping(1);
        break;
    }
  }

  char key = pad_get_key();

  interpret_pad_key(key);

  // front_Damping_Value = new_damping_values.substring(0,2).toInt();
  // rear_Damping_Value = new_damping_values.substring(2,4).toInt();
  // damping_update_flag = true;

  if (input_values.length() > 0) {
    display.showNumberDecEx(input_values.toInt(), 0b00001111<<min(4,input_values.length()), true);
  }else{
    display.showNumberDecEx(new_damping_values.toInt(), 0b00000000, true);
  }

  // Serial.print("Front Damping: ");
  // Serial.print(front_Damping_Value);
  // Serial.print(" Rear Damping: ");
  // Serial.print(rear_Damping_Value);
  // Serial.print(" FL Stall: ");
  // Serial.print(FL_Stall_State);
  // Serial.print(" FR Stall: ");
  // Serial.print(FR_Stall_State);
  // Serial.print(" RL Stall: ");
  // Serial.print(RL_Stall_State);
  // Serial.print(" RR Stall: ");
  // Serial.println(RR_Stall_State);


  // Update damping knobs
  update_damping_knobs();
}

