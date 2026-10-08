package frc.robot.Subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.ShooterConstants;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterTest {
  private final FakeIO io = new FakeIO();
  private final Shooter shooter = new Shooter(io, 100.0, 0.5);

  @BeforeEach
  void enableRobot() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void idleAndStopCommandAllOutputsToZero() {
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);
    shooter.start(3000.0);
    shooter.stop();
    assertEquals(0.0, io.flywheelTarget);
    assertEquals(0.0, io.feederOutput);
  }

  @Test
  void shootingCommandTargetsBothKrakensAndStopsOnRelease() {
    var command = shooter.shootCommand(() -> 3000.0);
    command.initialize();
    command.execute();
    shooter.periodic();
    assertEquals(3000.0, io.kraken1Target);
    assertEquals(3000.0, io.kraken2Target);

    command.end(true);
    assertEquals(0.0, io.kraken1Target);
    assertEquals(0.0, io.kraken2Target);
    assertEquals(0.0, io.feederOutput);
  }

  @Test
  void feederRequiresBothIndependentEncodersAtSpeedAndRecovers() {
    shooter.start(3000.0);
    io.setRpm(2800.0, 2800.0);
    shooter.periodic();
    assertEquals(3000.0, io.flywheelTarget);
    assertEquals(0.0, io.feederOutput);

    io.setRpm(3000.0, 3000.0);
    shooter.periodic();
    assertEquals(0.5, io.feederOutput);
    assertTrue(shooter.isReady());

    io.setRpm(3000.0, 2800.0);
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);
    assertFalse(shooter.isReady());

    io.setRpm(2800.0, 3000.0);
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);

    io.setRpm(3000.0, 3000.0);
    shooter.periodic();
    assertEquals(0.5, io.feederOutput);
  }

  @Test
  void invalidRequestEncoderFaultAndDisabledBlockFeeding() {
    shooter.start(Double.NaN);
    io.setRpm(3000.0, 3000.0);
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);

    shooter.start(3000.0);
    io.inputs.kraken2EncoderValid = false;
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);

    io.inputs.kraken2EncoderValid = true;
    io.inputs.fault = true;
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);

    io.inputs.fault = false;
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    shooter.periodic();
    assertEquals(0.0, io.feederOutput);
  }

  @Test
  void commissioningCommandsRunOnlyTheRequestedMotor() {
    var command = shooter.commissionKraken1Command(1000.0);
    command.initialize();
    shooter.periodic();
    assertEquals(1000.0, io.kraken1Target);
    assertEquals(0.0, io.kraken2Target);
    assertEquals(0.0, io.feederOutput);
    command.end(false);
    assertEquals(0.0, io.kraken1Target);

    assertThrows(IllegalArgumentException.class, () -> shooter.commissionFeederCommand(1.1));
  }

  @Test
  void realHardwareConfigurationIsBlockedUntilTodoValuesAreSupplied() {
    assertFalse(ShooterConstants.realHardwareConfigurationValid());
  }

  @Test
  void sharedFlywheelSimulationAcceleratesToReady() {
    var simulatedShooter = new Shooter(new ShooterIOSim(), ShooterConstants.SIM_RPM_TOLERANCE, 0.5);
    simulatedShooter.start(ShooterConstants.SIM_TEST_TARGET_RPM);
    for (int i = 0; i < 100; i++) simulatedShooter.periodic();
    assertTrue(simulatedShooter.isReady());
  }

  @Test
  void hubDistanceUsesAllianceAndRejectsUnknownInputs() {
    Pose2d blueHub =
        new Pose2d(
            Units.inchesToMeters(158.6), Units.inchesToMeters(317.7) / 2.0, new Rotation2d());
    Pose2d redHub =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6),
            Units.inchesToMeters(317.7) / 2.0,
            new Rotation2d());

    assertEquals(
        0.0,
        ShooterCalculator.distanceToHubMeters(
                blueHub, Optional.of(Alliance.Blue), new Translation2d())
            .orElseThrow(),
        1e-9);
    assertEquals(
        0.0,
        ShooterCalculator.distanceToHubMeters(
                redHub, Optional.of(Alliance.Red), new Translation2d())
            .orElseThrow(),
        1e-9);
    assertEquals(
        5.0,
        ShooterCalculator.distanceToHubMeters(
                new Pose2d(blueHub.getX() + 3.0, blueHub.getY() + 4.0, new Rotation2d()),
                Optional.of(Alliance.Blue),
                new Translation2d())
            .orElseThrow(),
        1e-9);
    assertTrue(
        ShooterCalculator.distanceToHubMeters(blueHub, Optional.empty(), new Translation2d())
            .isEmpty());
    assertTrue(
        ShooterCalculator.distanceToHubMeters(blueHub, Optional.of(Alliance.Blue)).isEmpty());
    assertTrue(
        ShooterCalculator.distanceToHubMeters(
                new Pose2d(Double.NaN, 0.0, new Rotation2d()),
                Optional.of(Alliance.Blue),
                new Translation2d())
            .isEmpty());
  }

  @Test
  void shooterExitOffsetRotatesWithRobotHeading() {
    Pose2d robotPose =
        new Pose2d(
            Units.inchesToMeters(158.6),
            Units.inchesToMeters(317.7) / 2.0 - 1.0,
            Rotation2d.fromDegrees(90.0));
    assertEquals(
        0.0,
        ShooterCalculator.distanceToHubMeters(
                robotPose, Optional.of(Alliance.Blue), new Translation2d(1.0, 0.0))
            .orElseThrow(),
        1e-9);
  }

  @Test
  void calculatorInterpolatesSyntheticCalibrationAndRejectsInvalidRanges() {
    var calibration = new InterpolatingDoubleTreeMap();
    calibration.put(2.0, 2000.0);
    calibration.put(4.0, 3000.0);

    assertEquals(2000.0, ShooterCalculator.calculateRPM(2.0, 2.0, 4.0, calibration));
    assertEquals(2500.0, ShooterCalculator.calculateRPM(3.0, 2.0, 4.0, calibration));
    assertTrue(Double.isNaN(ShooterCalculator.calculateRPM(1.0, 2.0, 4.0, calibration)));
    assertTrue(Double.isNaN(ShooterCalculator.calculateRPM(5.0, 2.0, 4.0, calibration)));
    assertTrue(Double.isNaN(ShooterCalculator.calculateRPM(2.0)));
  }

  @Test
  void rpmConvertsToPhoenixMotorRpsAtIoBoundary() {
    assertEquals(120.0, ShooterIOReal.flywheelRpmToMotorRps(3600.0, 2.0));
  }

  private static final class FakeIO implements ShooterIO {
    final ShooterIOInputs inputs = new ShooterIOInputs();
    double flywheelTarget;
    double kraken1Target;
    double kraken2Target;
    double feederOutput;

    FakeIO() {
      inputs.kraken1EncoderValid = true;
      inputs.kraken2EncoderValid = true;
    }

    void setRpm(double kraken1, double kraken2) {
      inputs.kraken1Rpm = kraken1;
      inputs.kraken2Rpm = kraken2;
    }

    @Override
    public void updateInputs(ShooterIOInputs destination) {
      destination.kraken1Rpm = inputs.kraken1Rpm;
      destination.kraken2Rpm = inputs.kraken2Rpm;
      destination.kraken1EncoderValid = inputs.kraken1EncoderValid;
      destination.kraken2EncoderValid = inputs.kraken2EncoderValid;
      destination.fault = inputs.fault;
    }

    @Override
    public void setFlywheelRpm(double targetRpm) {
      flywheelTarget = targetRpm;
      kraken1Target = targetRpm;
      kraken2Target = targetRpm;
    }

    @Override
    public void setKraken1Rpm(double targetRpm) {
      kraken1Target = targetRpm;
    }

    @Override
    public void setKraken2Rpm(double targetRpm) {
      kraken2Target = targetRpm;
    }

    @Override
    public void setFeederOutput(double output) {
      feederOutput = output;
    }

    @Override
    public void stop() {
      flywheelTarget = 0.0;
      kraken1Target = 0.0;
      kraken2Target = 0.0;
      feederOutput = 0.0;
    }
  }
}
