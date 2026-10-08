package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/** Spins the flywheel to the supplied target and stops the complete shooter when interrupted. */
public final class ShootCommand extends Command {
  private final Shooter shooter;
  private final DoubleSupplier targetRpmSupplier;

  public ShootCommand(Shooter shooter, DoubleSupplier targetRpmSupplier) {
    this.shooter = shooter;
    this.targetRpmSupplier = targetRpmSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.start(targetRpmSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
