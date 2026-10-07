package frc.robot.subsystems.drive.Motor.module;

import org.wpilib.math.geometry.Rotation2d;

public class ModuleIOSim implements ModuleIO {
  private double position, velocity;
  private Rotation2d angle = Rotation2d.ZERO;

  @Override
  public void updateInputs(ModuleIOInputs in) {
    in.drivePositionMeters = position;
    in.driveVelocityMetersPerSec = velocity;
    in.turnPosition = angle;
    in.connected = true;
    position += velocity * 0.02;
  }

  @Override
  public void setDesired(double speed, Rotation2d desired) {
    velocity = speed;
    angle = desired;
  }
}
