package frc.robot.Subsystems.drive.Motor.module;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionMeters;
    public double driveVelocityMetersPerSec;
    public Rotation2d turnPosition = Rotation2d.ZERO;
    public double turnVelocityRadPerSec;
    public boolean connected = true;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDesired(double metersPerSec, Rotation2d angle) {}
}
