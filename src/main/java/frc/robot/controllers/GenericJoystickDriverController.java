package frc.robot.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Joystick;
import org.wpilib.driverstation.POVDirection;

/** Generic joystick driver input. */
public final class GenericJoystickDriverController implements DriverController {
  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier twist;
  private final Supplier<POVDirection> pov;
  private final BooleanSupplier reset;

  public GenericJoystickDriverController(int port) {
    this(new Joystick(port));
  }

  GenericJoystickDriverController(Joystick joystick) {
    this(
        joystick::getX,
        joystick::getY,
        joystick::getTwist,
        joystick::getPOV,
        () -> joystick.getRawButton(1));
  }

  GenericJoystickDriverController(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier twist,
      Supplier<POVDirection> pov,
      BooleanSupplier reset) {
    this.x = x;
    this.y = y;
    this.twist = twist;
    this.pov = pov;
    this.reset = reset;
  }

  @Override
  public double forward() {
    double value = -y.getAsDouble();
    Logger.recordOutput("Drive/Joystick/Y", -value);
    return value;
  }

  @Override
  public double strafe() {
    double value = -x.getAsDouble();
    Logger.recordOutput("Drive/Joystick/X", -value);
    return value;
  }

  @Override
  public double rotation() {
    double value = -twist.getAsDouble();
    Logger.recordOutput("Drive/Joystick/Twist", -value);
    return value;
  }

  @Override
  public int pov() {
    return degrees(pov.get());
  }

  @Override
  public boolean reset() {
    return reset.getAsBoolean();
  }

  private static int degrees(POVDirection direction) {
    return direction
        .getAngle()
        .map(angle -> (int) Math.round(angle.getDegrees()))
        .orElse(POV_CENTER);
  }
}
