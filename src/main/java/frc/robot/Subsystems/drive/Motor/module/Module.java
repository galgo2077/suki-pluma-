package frc.robot.Subsystems.drive.Motor.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** One module in fixed drivetrain order: FL, FR, BL, BR. */
public final class Module {
  private final String name;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(String name, ModuleIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name, inputs);
  }

  public SwerveModuleState velocity() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }

  public SwerveModulePosition position() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }
}
