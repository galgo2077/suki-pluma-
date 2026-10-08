package frc.robot.Controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.XboxController;

/** Xbox driver input. */
public final class XboxDriverController implements DriverController {
  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final BooleanSupplier reset;
  private final BooleanSupplier aimToggle;

  public XboxDriverController(int port) {
    this(new XboxController(port));
  }

  XboxDriverController(XboxController xbox) {
    this(xbox::getLeftX, xbox::getLeftY, xbox::getRightX, xbox::getYButton, xbox::getAButton);
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
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.reset = reset;
    this.aimToggle = aimToggle;
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
    return aimToggle.getAsBoolean();
  }
}
