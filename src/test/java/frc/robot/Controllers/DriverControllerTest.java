package frc.robot.Controllers;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DriverControllerTest {
  @Test
  void xboxAndJoystickNormalizeToTheSameInputs() {
    DriverController xbox =
        new XboxDriverController(() -> 0.25, () -> -0.5, () -> -0.75, () -> false);
    assertInputs(xbox);

    DriverController joystick =
        new GenericJoystickDriverController(() -> 0.25, () -> -0.5, () -> -0.75, () -> false);
    assertInputs(joystick);
  }

  private static void assertInputs(DriverController controller) {
    assertEquals(-0.5, controller.forward(), 1e-9);
    assertEquals(0.25, controller.strafe(), 1e-9);
    assertEquals(0.75, controller.rotation(), 1e-9);
  }
}
