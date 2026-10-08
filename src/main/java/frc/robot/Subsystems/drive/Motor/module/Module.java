package frc.robot.Subsystems.drive.Motor.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** One module in fixed drivetrain order: FL, FR, BL, BR. */
public final class Module {
  private final String name;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private Rotation2d turnSetpoint = new Rotation2d();

  public Module(String name, ModuleIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name, inputs);
    Logger.recordOutput("Drive/" + name + "/TurnSetpoint", turnSetpoint);
  }

  public SwerveModuleState velocity() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }

  public SwerveModulePosition position() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }

  public void runSetpoint(SwerveModuleState setpoint) {
    setpoint.optimize(inputs.turnPosition);
    turnSetpoint = setpoint.angle;
    io.setDesired(setpoint.speedMetersPerSecond, setpoint.angle);
  }
}
