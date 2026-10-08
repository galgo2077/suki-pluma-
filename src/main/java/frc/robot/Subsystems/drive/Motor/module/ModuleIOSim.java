package frc.robot.Subsystems.drive.Motor.module;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOSim implements ModuleIO {
  private double position, velocity;
  private Rotation2d angle = new Rotation2d();

  @Override
  public void updateInputs(ModuleIOInputs in) {
    in.drivePositionMeters = position;
    in.driveVelocityMetersPerSec = velocity;
    in.turnPosition = angle;
    in.connected = true;
    position += velocity * 0.02; // SIM ONLY: fixed simulation period
  }

  @Override
  public void setDesired(double speed, Rotation2d desired) {
    velocity = speed;
    angle = desired;
  }
}
