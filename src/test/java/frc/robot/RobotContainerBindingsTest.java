package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SystemConstants;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class RobotContainerBindingsTest {
  @Test
  void realShooterRequiresValidHardwareConfiguration() {
    assertFalse(RobotContainer.shootingPermitted(SystemConstants.Mode.REAL, false));
    assertTrue(RobotContainer.shootingPermitted(SystemConstants.Mode.REAL, true));
  }

  @Test
  void simIsIndependentAndReplayNeverShoots() {
    assertTrue(RobotContainer.shootingPermitted(SystemConstants.Mode.SIM, false));
    assertFalse(RobotContainer.shootingPermitted(SystemConstants.Mode.REPLAY, true));
  }

  @Test
  void xboxRightTriggerUsesDigitalThreshold() {
    var xbox = new CommandXboxController(SystemConstants.XBOX_PORT);
    Trigger trigger = xbox.rightTrigger(SystemConstants.XBOX_TRIGGER_THRESHOLD);
    DriverStationSim.setJoystickAxisCount(SystemConstants.XBOX_PORT, 6);
    DriverStationSim.setJoystickIsXbox(SystemConstants.XBOX_PORT, true);
    DriverStationSim.setJoystickAxis(
        SystemConstants.XBOX_PORT, XboxController.Axis.kRightTrigger.value, 0.49);
    DriverStationSim.notifyNewData();
    assertFalse(trigger.getAsBoolean());
    DriverStationSim.setJoystickAxis(
        SystemConstants.XBOX_PORT, XboxController.Axis.kRightTrigger.value, 0.51);
    DriverStationSim.notifyNewData();
    assertTrue(trigger.getAsBoolean());
  }

  @Test
  void combinedShooterTriggerRemainsActiveUntilBothInputsRelease() {
    var joystick = new AtomicBoolean();
    var xbox = new AtomicBoolean();
    Trigger trigger = RobotContainer.combinedShootTrigger(joystick::get, new Trigger(xbox::get));
    assertFalse(trigger.getAsBoolean());
    joystick.set(true);
    assertTrue(trigger.getAsBoolean());
    xbox.set(true);
    joystick.set(false);
    assertTrue(trigger.getAsBoolean());
    xbox.set(false);
    assertFalse(trigger.getAsBoolean());
  }
}
