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
#include <SpeedyStepper.h>

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE     35 // [0..255]
#define MICROSTEPS       8 // 8, 16, 32, 64 or 256

#define EN_PIN           8 // Enable
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
SpeedyStepper FL_stepper;
SpeedyStepper FR_stepper;
SpeedyStepper RL_stepper;
SpeedyStepper RR_stepper;

using namespace TMC2209_n;

// Pin connected to the signal you want to detect
const int FL_Stall_Pin = 18;

// Variable to store the state of the signal
volatile bool FL_Stall_State = false;

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  // Set the signal pin as input
  pinMode(FL_Stall_Pin, INPUT);

  // Attach the interrupt to the rising edge of the signal
  attachInterrupt(digitalPinToInterrupt(FL_Stall_Pin), handleInterrupt, RISING);

  SERIAL_PORT.begin(115200);
  setup_tmc_driver(FL_driver);
  setup_tmc_driver(FR_driver);
  setup_tmc_driver(RL_driver);
  setup_tmc_driver(RR_driver);

  FL_stepper.connectToPins(FL_STEP_PIN, FL_DIR_PIN);
  FR_stepper.connectToPins(FR_STEP_PIN, FR_DIR_PIN);
  RL_stepper.connectToPins(RL_STEP_PIN, RL_DIR_PIN);
  RR_stepper.connectToPins(RR_STEP_PIN, RR_DIR_PIN);
  
  FL_stepper.setStepsPerRevolution(200*MICROSTEPS);
  FL_stepper.setSpeedInRevolutionsPerSecond(0.5);
  FL_stepper.setAccelerationInRevolutionsPerSecondPerSecond(4);
  FR_stepper.setStepsPerRevolution(200*MICROSTEPS);
  FR_stepper.setSpeedInRevolutionsPerSecond(0.5);
  FR_stepper.setAccelerationInRevolutionsPerSecondPerSecond(4);
  RL_stepper.setStepsPerRevolution(200*MICROSTEPS);
  RL_stepper.setSpeedInRevolutionsPerSecond(1);
  RL_stepper.setAccelerationInRevolutionsPerSecondPerSecond(4);
  RR_stepper.setStepsPerRevolution(200*MICROSTEPS);
  RR_stepper.setSpeedInRevolutionsPerSecond(1);
  RR_stepper.setAccelerationInRevolutionsPerSecondPerSecond(4);
  
  FL_stepper.setupRelativeMoveInRevolutions(-3.6);
  FR_stepper.setupRelativeMoveInRevolutions(-3.6);
  RL_stepper.setupRelativeMoveInRevolutions(-2);
  RR_stepper.setupRelativeMoveInRevolutions(-2);
  while((!FL_stepper.motionComplete()) || (!FR_stepper.motionComplete()))
  {
    FL_stepper.processMovement();
    FR_stepper.processMovement();
    RL_stepper.processMovement();
    RR_stepper.processMovement();
  }
  delay(2000);
  
  FL_stepper.setupRelativeMoveInRevolutions(10);
}

void loop() {
  //
  // setup the speed, acceleration and number of steps to move for the 
  // X motor, note: these commands do not start moving yet
  //
  int dir = 1;
  // FL_stepper.setupRelativeMoveInRevolutions(3.6*dir);
  // FR_stepper.setupRelativeMoveInRevolutions(3.6*dir);
  // RL_stepper.setupRelativeMoveInRevolutions(2);
  // RR_stepper.setupRelativeMoveInRevolutions(2);


  //
  // now execute the moves, looping until both motors have finished
  //
  while(!FL_Stall_State && ((!FL_stepper.motionComplete()) || (!FR_stepper.motionComplete())))
  {
    FL_stepper.processMovement();
    FR_stepper.processMovement();
    RL_stepper.processMovement();
    RR_stepper.processMovement();
  }
  if (FL_Stall_State) {
    Serial.println("Stall detected");
    FL_stepper.setupStop();
    FR_stepper.setupStop();
    RL_stepper.setupStop();
    RR_stepper.setupStop();
    FL_Stall_State = false;
  }
  dir = -1*dir;


  //
  // now that the rotations have finished, delay 1 second before starting 
  // the next move
  //
  // delay(1000);
}

