package frc.robot.Constants;

import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class DriveHardwareConfigurationTest {
  @Test
  void realHardwareStaysBlockedUntilAllSafetyValuesAreVerified() {
    assertFalse(DriveConstants.realHardwareConfigurationValid());
  }
}
