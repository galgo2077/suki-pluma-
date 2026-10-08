package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

class RobotContainerTest {
  @Test
  void configuresDriveCommandAsDefault() {
    var container = new RobotContainer();
    assertSame(
        container.defaultDriveCommand(),
        container.defaultDriveCommand().getRequirements().iterator().next().getDefaultCommand());
  }

  @Test
  void xboxATogglesAimWithoutInterruptingOnRelease() {
    DriverStationSim.resetData();
    DriverStationSim.setJoystickButtonCount(0, 1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    var container = new RobotContainer();
    var scheduler = CommandScheduler.getInstance();
    scheduler.run();
    assertTrue(scheduler.isScheduled(container.defaultDriveCommand()));

    DriverStationSim.setJoystickButton(0, 1, true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertFalse(scheduler.isScheduled(container.defaultDriveCommand()));

    DriverStationSim.setJoystickButton(0, 1, false);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertFalse(scheduler.isScheduled(container.defaultDriveCommand()));

    DriverStationSim.setJoystickButton(0, 1, true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertTrue(scheduler.isScheduled(container.defaultDriveCommand()));
    scheduler.cancelAll();
  }
}
