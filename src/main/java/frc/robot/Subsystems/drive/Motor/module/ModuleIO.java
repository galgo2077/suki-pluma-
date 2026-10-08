package frc.robot.Subsystems.drive.Motor.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionMeters;
    public double driveVelocityMetersPerSec;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec;
    public boolean connected = true;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDesired(double metersPerSec, Rotation2d angle) {}
}
