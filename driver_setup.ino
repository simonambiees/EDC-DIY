void setup_tmc_driver(TMC2209Stepper &driver){
  // driver.beginSerial(115200);
  driver.begin();
  // driver.toff(4);
  // driver.blank_time(24);
  driver.rms_current(1600); // mA
  driver.pwm_autoscale(1);
  driver.microsteps(MICROSTEPS);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  // driver.TPWMTHRS(0x1); // 20bit max
  // driver.semin(0);
  // driver.semin(5);
  // driver.semax(2);
  // driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);
  // driver.en_spreadCycle(true);
}

