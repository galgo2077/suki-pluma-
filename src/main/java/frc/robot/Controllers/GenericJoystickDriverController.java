package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Generic joystick driver input. */
public final class GenericJoystickDriverController implements DriverController {
  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier twist;
  private final BooleanSupplier reset;

  public GenericJoystickDriverController(int port) {
    this(new Joystick(port));
  }

  GenericJoystickDriverController(Joystick joystick) {
    this(joystick::getX, joystick::getY, joystick::getTwist, () -> joystick.getRawButton(1));
  }

  GenericJoystickDriverController(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, BooleanSupplier reset) {
    this.x = x;
    this.y = y;
    this.twist = twist;
    this.reset = reset;
  }

  @Override
  public double forward() {
    double value = y.getAsDouble();
    Logger.recordOutput("Drive/Joystick/Y", value);
    return value;
  }

  @Override
  public double strafe() {
    double value = x.getAsDouble();
    Logger.recordOutput("Drive/Joystick/X", value);
    return value;
  }

  @Override
  public double rotation() {
    double value = -twist.getAsDouble();
    Logger.recordOutput("Drive/Joystick/Twist", -value);
    return value;
  }

  @Override
  public boolean reset() {
    return reset.getAsBoolean();
  }
}
