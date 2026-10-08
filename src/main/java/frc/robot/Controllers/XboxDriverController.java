package frc.robot.Controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Xbox driver input. */
public final class XboxDriverController implements DriverController {
  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final BooleanSupplier reset;
  private final BooleanSupplier aimToggle;
  private final BooleanSupplier connected;

  public XboxDriverController(int port) {
    this(
        new XboxController(port),
        () ->
            DriverStation.isJoystickConnected(port) && DriverStation.getStickButtonCount(port) > 0);
  }

  XboxDriverController(XboxController xbox) {
    this(xbox, () -> true);
  }

  private XboxDriverController(XboxController xbox, BooleanSupplier connected) {
    this(
        xbox::getLeftX,
        xbox::getLeftY,
        () -> xbox.getRawAxis(3),
        xbox::getYButton,
        xbox::getAButton,
        connected);
  }

  XboxDriverController(
      DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, BooleanSupplier reset) {
    this(leftX, leftY, rightX, reset, () -> false);
  }

  XboxDriverController(
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      BooleanSupplier reset,
      BooleanSupplier aimToggle) {
    this(leftX, leftY, rightX, reset, aimToggle, () -> true);
  }

  XboxDriverController(
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      BooleanSupplier reset,
      BooleanSupplier aimToggle,
      BooleanSupplier connected) {
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.reset = reset;
    this.aimToggle = aimToggle;
    this.connected = connected;
  }

  @Override
  public double forward() {
    double value = leftY.getAsDouble();
    Logger.recordOutput("Drive/Xbox/LeftY", value);
    return value;
  }

  @Override
  public double strafe() {
    double value = leftX.getAsDouble();
    Logger.recordOutput("Drive/Xbox/LeftX", value);
    return value;
  }

  @Override
  public double rotation() {
    double value = -rightX.getAsDouble();
    Logger.recordOutput("Drive/Xbox/RightX", -value);
    return value;
  }

  @Override
  public boolean reset() {
    return reset.getAsBoolean();
  }

  @Override
  public boolean aimToggle() {
    return connected.getAsBoolean() && aimToggle.getAsBoolean();
  }
}
