package frc.robot.subsystems.drive.Motor.module;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/** One module in fixed drivetrain order: FL, FR, BL, BR. */
public final class Module {
  private final String name;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private Rotation2d turnSetpoint = Rotation2d.ZERO;

  public Module(String name, ModuleIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name, inputs);
    Logger.recordOutput("Drive/" + name + "/TurnSetpoint", turnSetpoint);
  }

  public SwerveModuleVelocity velocity() {
    return new SwerveModuleVelocity(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }

  public SwerveModulePosition position() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }

  public void runSetpoint(SwerveModuleVelocity setpoint) {
    var optimized = setpoint.optimize(inputs.turnPosition);
    turnSetpoint = optimized.angle;
    io.setDesired(optimized.velocity, optimized.angle);
  }
}
