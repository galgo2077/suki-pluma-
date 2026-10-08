package frc.robot.Constants;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PathplannerConstantsTest {
  @Test
  void blocksAutonomousUntilPhysicalConfigurationIsMeasured() {
    assertTrue(PathplannerConstants.robotConfig().isEmpty());
  }
}
