void setup_tmc_driver(TMC2209Stepper driver){
  driver.beginSerial(115200);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(800); // mA
  driver.microsteps(MICROSTEPS);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);
}