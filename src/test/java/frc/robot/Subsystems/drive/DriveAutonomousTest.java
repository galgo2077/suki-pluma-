package frc.robot.Subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import java.util.Arrays;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(OrderAnnotation.class)
class DriveAutonomousTest {
  @Test
  @Order(1)
  void configuresSimulationWithTestOnlyValues() {
    RobotContainer container = new RobotContainer();

    SmartDashboard.updateValues();
    assertTrue(SmartDashboard.getBoolean("AutoBuilder Ready", false));
    assertEquals("Ready", SmartDashboard.getString("AutoBuilder Status", ""));
    assertTrue(
        NetworkTableInstance.getDefault().getTopic("/SmartDashboard/AutoBuilder Ready").exists());
    assertTrue(
        NetworkTableInstance.getDefault().getTopic("/SmartDashboard/AutoBuilder Status").exists());
    assertNotNull(container.getAutonomousCommand());
  }

  @Test
  @Order(2)
  void publishesChooserForControlledRobotConfig() {
    DriveAutonomous.configure(new Drive(), testRobotConfig());

    new RobotContainer();
    SmartDashboard.updateValues();

    assertTrue(SmartDashboard.getBoolean("AutoBuilder Ready", false));
    assertEquals("Ready", SmartDashboard.getString("AutoBuilder Status", ""));
    assertNotNull(SmartDashboard.getData("Auto Chooser"));
    assertTrue(
        NetworkTableInstance.getDefault().getTopic("/SmartDashboard/Auto Chooser/.name").exists());
    assertTrue(
        Arrays.asList(
                NetworkTableInstance.getDefault()
                    .getStringArrayTopic("/SmartDashboard/Auto Chooser/options")
                    .subscribe(new String[0])
                    .get())
            .contains("New Auto"));
  }

  private static RobotConfig testRobotConfig() {
    ModuleConfig moduleConfig = new ModuleConfig(0.05, 1.0, 1.0, DCMotor.getNEO(1), 6.0, 40.0, 1);
    return new RobotConfig(
        50.0,
        5.0,
        moduleConfig,
        new Translation2d(0.25, 0.25),
        new Translation2d(0.25, -0.25),
        new Translation2d(-0.25, 0.25),
        new Translation2d(-0.25, -0.25));
  }
}
