package frc.robot.Controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.driverstation.XboxController;

/** Xbox driver input. */
public final class XboxDriverController implements DriverController {
  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final Supplier<POVDirection> pov;
  private final BooleanSupplier reset;

  public XboxDriverController(int port) {
    this(new XboxController(port));
  }

  XboxDriverController(XboxController xbox) {
    this(
        xbox::getLeftX,
        xbox::getLeftY,
        xbox::getRightX,
        () -> xbox.getHID().getPOV(),
        xbox::getYButton);
  }

  XboxDriverController(
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      Supplier<POVDirection> pov,
      BooleanSupplier reset) {
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.pov = pov;
    this.reset = reset;
  }

  @Override
  public double forward() {
    double value = -leftY.getAsDouble();
    Logger.recordOutput("Drive/Xbox/LeftY", -value);
    return value;
  }

  @Override
  public double strafe() {
    double value = -leftX.getAsDouble();
    Logger.recordOutput("Drive/Xbox/LeftX", -value);
    return value;
  }

  @Override
  public double rotation() {
    double value = -rightX.getAsDouble();
    Logger.recordOutput("Drive/Xbox/RightX", -value);
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
