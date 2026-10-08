package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;

/** REAL implementation. Construction requires controller IDs and inversion settings. */
public final class ShooterIOReal implements ShooterIO {
  private final TalonFX kraken1;
  private final TalonFX kraken2;
  private final SparkMax feeder;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public ShooterIOReal() {
    if (!ShooterConstants.realHardwareConfigurationValid())
      throw new IllegalStateException("Shooter REAL hardware configuration is incomplete");
    kraken1 = new TalonFX(ShooterConstants.KRAKEN_1_CAN_ID);
    kraken2 = new TalonFX(ShooterConstants.KRAKEN_2_CAN_ID);
    feeder = new SparkMax(ShooterConstants.FEEDER_CAN_ID, MotorType.kBrushless);
    configureKraken(kraken1, ShooterConstants.KRAKEN_1_INVERTED);
    configureKraken(kraken2, ShooterConstants.KRAKEN_2_INVERTED);
    feeder.configure(
        new SparkMaxConfig().inverted(ShooterConstants.FEEDER_INVERTED),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.kraken1Rpm =
        kraken1.getVelocity().getValueAsDouble()
            / ShooterConstants.MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION
            * 60.0;
    inputs.kraken2Rpm =
        kraken2.getVelocity().getValueAsDouble()
            / ShooterConstants.MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION
            * 60.0;
    inputs.kraken1Current = kraken1.getSupplyCurrent().getValueAsDouble();
    inputs.kraken2Current = kraken2.getSupplyCurrent().getValueAsDouble();
    inputs.feederCurrent = feeder.getOutputCurrent();
    inputs.kraken1EncoderValid = kraken1.isConnected();
    inputs.kraken2EncoderValid = kraken2.isConnected();
    inputs.fault =
        kraken1.getFaultField().getValue() != 0
            || kraken2.getFaultField().getValue() != 0
            || feeder.hasActiveFault();
  }

  @Override
  public void setFlywheelRpm(double targetRpm) {
    setKraken1Rpm(targetRpm);
    setKraken2Rpm(targetRpm);
  }

  @Override
  public void setKraken1Rpm(double targetRpm) {
    kraken1.setControl(
        velocityRequest.withVelocity(
            flywheelRpmToMotorRps(
                targetRpm, ShooterConstants.MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION)));
  }

  @Override
  public void setKraken2Rpm(double targetRpm) {
    kraken2.setControl(
        velocityRequest.withVelocity(
            flywheelRpmToMotorRps(
                targetRpm, ShooterConstants.MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION)));
  }

  static double flywheelRpmToMotorRps(
      double flywheelRpm, double motorRotationsPerFlywheelRotation) {
    return flywheelRpm / 60.0 * motorRotationsPerFlywheelRotation;
  }

  private static void configureKraken(TalonFX motor, boolean inverted) {
    var configuration = new TalonFXConfiguration();
    configuration.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    if (finite(ShooterConstants.KRAKEN_KP)) configuration.Slot0.kP = ShooterConstants.KRAKEN_KP;
    if (finite(ShooterConstants.KRAKEN_KI)) configuration.Slot0.kI = ShooterConstants.KRAKEN_KI;
    if (finite(ShooterConstants.KRAKEN_KD)) configuration.Slot0.kD = ShooterConstants.KRAKEN_KD;
    if (finite(ShooterConstants.KRAKEN_KS)) configuration.Slot0.kS = ShooterConstants.KRAKEN_KS;
    if (finite(ShooterConstants.KRAKEN_KV)) configuration.Slot0.kV = ShooterConstants.KRAKEN_KV;
    if (!motor.getConfigurator().apply(configuration).isOK())
      throw new IllegalStateException("Failed to configure shooter Kraken");
  }

  private static boolean finite(Double value) {
    return value != null && Double.isFinite(value);
  }

  @Override
  public void setFeederOutput(double output) {
    feeder.set(output);
  }

  @Override
  public void stop() {
    kraken1.stopMotor();
    kraken2.stopMotor();
    feeder.stopMotor();
  }
}
