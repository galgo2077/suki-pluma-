package frc.robot.Subsystems.shooter;

import frc.robot.Constants.ShooterConstants;

/** Independent flywheel feedback channels for shooter simulation. */
public final class ShooterIOSim implements ShooterIO {
  private double kraken1TargetRpm;
  private double kraken2TargetRpm;
  private double kraken1Rpm;
  private double kraken2Rpm;
  private double feederOutput;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    kraken1Rpm +=
        (kraken1TargetRpm - kraken1Rpm)
            * 0.02
            / ShooterConstants.SIM_FLYWHEEL_TIME_CONSTANT_SECONDS;
    kraken2Rpm +=
        (kraken2TargetRpm - kraken2Rpm)
            * 0.02
            / ShooterConstants.SIM_FLYWHEEL_TIME_CONSTANT_SECONDS;
    inputs.kraken1Rpm = kraken1Rpm;
    inputs.kraken2Rpm = kraken2Rpm;
    inputs.kraken1EncoderValid = true;
    inputs.kraken2EncoderValid = true;
  }

  @Override
  public void setFlywheelRpm(double targetRpm) {
    setKraken1Rpm(targetRpm);
    setKraken2Rpm(targetRpm);
  }

  @Override
  public void setKraken1Rpm(double targetRpm) {
    kraken1TargetRpm = targetRpm;
  }

  @Override
  public void setKraken2Rpm(double targetRpm) {
    kraken2TargetRpm = targetRpm;
  }

  @Override
  public void setFeederOutput(double output) {
    feederOutput = output;
  }

  @Override
  public void stop() {
    kraken1TargetRpm = 0.0;
    kraken2TargetRpm = 0.0;
    feederOutput = 0.0;
  }
}
