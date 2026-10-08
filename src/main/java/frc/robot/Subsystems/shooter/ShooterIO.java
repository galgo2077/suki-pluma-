package frc.robot.Subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the complete two-Kraken flywheel and NEO feeder mechanism. */
public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double kraken1Rpm;
    public double kraken2Rpm;
    public double kraken1Current;
    public double kraken2Current;
    public double feederCurrent;
    public boolean kraken1EncoderValid;
    public boolean kraken2EncoderValid;
    public boolean fault;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelRpm(double targetRpm) {}

  default void setKraken1Rpm(double targetRpm) {}

  default void setKraken2Rpm(double targetRpm) {}

  default void setFeederOutput(double output) {}

  default void stop() {}
}
