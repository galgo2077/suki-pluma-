package frc.robot;

import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;

class RobotContainerTest {
  @Test
  void configuresDriveCommandAsDefault() {
    var container = new RobotContainer();
    assertSame(
        container.defaultDriveCommand(),
        container.defaultDriveCommand().requirements().iterator().next().getDefaultCommand());
  }
}
