package frc.robot.Subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Interlocks feeding on independently measured speed from both flywheel motors. */
public final class Shooter extends SubsystemBase {
  public enum State {
    IDLE,
    SPINNING_UP,
    READY,
    RECOVERING,
    FAULT
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final double toleranceRpm;
  private final double feederOutput;
  private boolean requestActive;
  private boolean calculationValid;
  private boolean commissioningActive;
  private double targetRpm;
  private double distanceToHubMeters = Double.NaN;
  private double commissioningKraken1Rpm;
  private double commissioningKraken2Rpm;
  private double commissioningFeederOutput;
  private boolean realMode;
  private boolean configurationValid;
  private State state = State.IDLE;

  public Shooter(ShooterIO io, double toleranceRpm, double feederOutput) {
    this.io = io;
    this.toleranceRpm = toleranceRpm;
    this.feederOutput = feederOutput;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    boolean validTarget = Double.isFinite(targetRpm) && targetRpm > 0.0;
    boolean atSpeed =
        requestActive
            && validTarget
            && inputs.kraken1EncoderValid
            && inputs.kraken2EncoderValid
            && !inputs.fault
            && Math.abs(inputs.kraken1Rpm - targetRpm) <= toleranceRpm
            && Math.abs(inputs.kraken2Rpm - targetRpm) <= toleranceRpm;
    boolean enabled = !DriverStation.isDisabled();
    boolean feederRunning = false;
    double commandedFeederOutput = 0.0;

    if (!enabled || inputs.fault) {
      io.stop();
      commissioningActive = false;
      state = inputs.fault ? State.FAULT : State.IDLE;
    } else if (commissioningActive) {
      io.setKraken1Rpm(commissioningKraken1Rpm);
      io.setKraken2Rpm(commissioningKraken2Rpm);
      io.setFeederOutput(commissioningFeederOutput);
      feederRunning = commissioningFeederOutput != 0.0;
      commandedFeederOutput = commissioningFeederOutput;
      state = State.IDLE;
    } else if (!requestActive || !validTarget) {
      io.stop();
      state = State.IDLE;
    } else {
      io.setFlywheelRpm(targetRpm);
      feederRunning = atSpeed;
      commandedFeederOutput = feederRunning ? feederOutput : 0.0;
      io.setFeederOutput(commandedFeederOutput);
      state =
          atSpeed
              ? State.READY
              : (state == State.READY || state == State.RECOVERING
                  ? State.RECOVERING
                  : State.SPINNING_UP);
    }

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/DistanceToHubMeters", distanceToHubMeters);
    Logger.recordOutput("Shooter/TargetRPM", targetRpm);
    Logger.recordOutput("Shooter/Kraken1RPM", inputs.kraken1Rpm);
    Logger.recordOutput("Shooter/Kraken2RPM", inputs.kraken2Rpm);
    Logger.recordOutput("Shooter/AtSpeed", atSpeed);
    Logger.recordOutput("Shooter/FeederRunning", feederRunning);
    Logger.recordOutput("Shooter/CalculationValid", calculationValid);
    Logger.recordOutput("Shooter/FeederOutput", commandedFeederOutput);
    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/Kraken1Current", inputs.kraken1Current);
    Logger.recordOutput("Shooter/Kraken2Current", inputs.kraken2Current);
    Logger.recordOutput("Shooter/FeederCurrent", inputs.feederCurrent);
    Logger.recordOutput("Shooter/Fault", inputs.fault);
    Logger.recordOutput("Shooter/RealEnabled", realMode);
    Logger.recordOutput("Shooter/ConfigurationValid", configurationValid);
    Logger.recordOutput("Shooter/CommissioningActive", commissioningActive);
  }

  public void setTargetRpm(double rpm) {
    targetRpm = rpm;
  }

  public void setDistanceToHubMeters(double distanceMeters) {
    distanceToHubMeters = distanceMeters;
  }

  public void setCalculationValid(boolean calculationValid) {
    this.calculationValid = calculationValid;
  }

  public void setConfigurationStatus(boolean realMode, boolean configurationValid) {
    this.realMode = realMode;
    this.configurationValid = configurationValid;
  }

  public void start(double rpm) {
    commissioningActive = false;
    targetRpm = rpm;
    requestActive = true;
  }

  public void stop() {
    requestActive = false;
    commissioningActive = false;
    targetRpm = 0.0;
    io.stop();
  }

  public boolean isReady() {
    return state == State.READY;
  }

  public Command commissionKraken1Command(double rpm) {
    return startEnd(() -> startCommissioning(rpm, 0.0, 0.0), this::stop)
        .withName("CommissionKraken1");
  }

  public Command commissionKraken2Command(double rpm) {
    return startEnd(() -> startCommissioning(0.0, rpm, 0.0), this::stop)
        .withName("CommissionKraken2");
  }

  public Command commissionFeederCommand(double output) {
    if (!Double.isFinite(output) || Math.abs(output) > 1.0)
      throw new IllegalArgumentException("Commissioning feeder output must be between -1 and 1");
    return startEnd(() -> startCommissioning(0.0, 0.0, output), this::stop)
        .withName("CommissionFeeder");
  }

  private void startCommissioning(double kraken1Rpm, double kraken2Rpm, double feederOutput) {
    if (!Double.isFinite(kraken1Rpm) || !Double.isFinite(kraken2Rpm))
      throw new IllegalArgumentException("Commissioning RPM must be finite");
    requestActive = false;
    commissioningKraken1Rpm = kraken1Rpm;
    commissioningKraken2Rpm = kraken2Rpm;
    commissioningFeederOutput = feederOutput;
    commissioningActive = true;
  }
}
