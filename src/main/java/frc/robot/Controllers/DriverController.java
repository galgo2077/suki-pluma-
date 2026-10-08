package frc.robot.Controllers;

/** Normalized driver input independent of the physical HID. */
public interface DriverController {
  int POV_CENTER = -1;

  double forward();

  double strafe();

  double rotation();

  int pov();

  boolean reset();
}
