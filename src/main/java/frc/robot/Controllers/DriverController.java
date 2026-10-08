package frc.robot.Controllers;

/** Normalized driver input independent of the physical HID. */
public interface DriverController {
  double forward();

  double strafe();

  double rotation();

  boolean reset();

  /** Xbox A shooter-target aim toggle; generic joysticks intentionally have no binding. */
  default boolean aimToggle() {
    return false;
  }
}
