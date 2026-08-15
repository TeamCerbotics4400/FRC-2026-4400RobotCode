package frc.robot.subsystems.Hooper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HooperContants;

/**
 * HooperIO implementation for the real robot. The upper roller is the leader, the lower roller
 * follows it inverted, and the "parrilla" roller is controlled independently.
 */
public class HooperFX implements HooperIO {

  private final TalonFX upperRoller = new TalonFX(HooperContants.upperRollerID, "canivore");
  private final TalonFX lowerRoller = new TalonFX(HooperContants.lowerRollerID, "canivore");

  // Parrilla (grid roller feeding the shooter)
  private final TalonFX parrilla = new TalonFX(HooperContants.parrillaID, "canivore");

  // Status signals refreshed together for synchronous telemetry
  private final StatusSignal<AngularVelocity> upperVel = upperRoller.getVelocity();
  private final StatusSignal<AngularVelocity> parrillaVel = parrilla.getVelocity();

  private final StatusSignal<Voltage> upperVolts = upperRoller.getMotorVoltage();
  private final StatusSignal<Voltage> parrillaVolts = parrilla.getMotorVoltage();

  private final StatusSignal<Current> upperCurrent = upperRoller.getStatorCurrent();
  private final StatusSignal<Current> parrillaCurrent = parrilla.getStatorCurrent();

  // Per-motor configurations
  private final TalonFXConfiguration upperRollerConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration parrillaRollerConfig = new TalonFXConfiguration();

  // Control request with FOC enabled
  private final MotionMagicVelocityVoltage m_request =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut m_stop = new NeutralOut();

  private double upperTargetRPS = 0.0;
  private double lowerTargetRPS = 0.0;

  public HooperFX() {
    // --- Upper Roller configuration ---
    upperRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    upperRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    upperRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    upperRoller.setNeutralMode(NeutralModeValue.Coast);

    upperRollerConfig.Slot0.kP = 0.12; // Tune as needed
    upperRollerConfig.Slot0.kV = 0.11;
    upperRollerConfig.MotionMagic.MotionMagicAcceleration = 100;
    upperRollerConfig.MotionMagic.MotionMagicJerk = 1000;

    // --- Lower Roller configuration ---
    parrillaRollerConfig.CurrentLimits.StatorCurrentLimit = 80;
    parrillaRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    parrillaRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    parrillaRollerConfig.Slot0.kP = 10;
    parrillaRollerConfig.Slot0.kV = 10;
    parrillaRollerConfig.MotionMagic.MotionMagicAcceleration = 100;
    parrillaRollerConfig.MotionMagic.MotionMagicJerk = 1000;

    // Lower roller mirrors the upper one in the opposite direction, so balls are pinched upward.
    lowerRoller.setControl(new Follower(HooperContants.upperRollerID, MotorAlignmentValue.Opposed));
    lowerRoller.setNeutralMode(NeutralModeValue.Coast);

    parrilla.setNeutralMode(NeutralModeValue.Coast);

    upperRollerConfig.CurrentLimits.SupplyCurrentLimit = 50;
    parrillaRollerConfig.CurrentLimits.SupplyCurrentLimit = 50;

    upperRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    parrillaRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations through each motor's Configurator
    upperRoller.getConfigurator().apply(upperRollerConfig);
    parrilla.getConfigurator().apply(parrillaRollerConfig);

    // Raise the refresh rate to 250 Hz (4 ms) only on motor ID 40 (Upper Roller)
    // for very fast jam detection.
    upperVel.setUpdateFrequency(250.0);
    upperVolts.setUpdateFrequency(250.0);
    upperCurrent.setUpdateFrequency(250.0);
  }

  @Override
  public void updateInputs(HooperRPMSInputs inputs) {
    // One CAN round trip for all signals, so every value comes from the same instant.
    BaseStatusSignal.refreshAll(
        upperVel, parrillaVel, upperVolts, parrillaVolts, upperCurrent, parrillaCurrent);

    // Phoenix 6 reports rotations per second (RPS), convert to RPM for the Logger
    inputs.upperRollerRPMs = upperVel.getValueAsDouble() * 60.0;
    inputs.parrillaRollerRPMs = parrillaVel.getValueAsDouble() * 60.0;

    inputs.upperTargetRPM = upperTargetRPS * 60.0;
    inputs.parrilaTargetRPM = lowerTargetRPS * 60.0;

    inputs.upperAppliedVolts = upperVolts.getValueAsDouble();
    inputs.parrilaAppliedVolts = parrillaVolts.getValueAsDouble();

    inputs.upperStatorCurrent = upperCurrent.getValueAsDouble();
    inputs.parrillaStatorCurrent = parrillaCurrent.getValueAsDouble();

    inputs.upperStatorCurrent = upperRoller.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setTargets(double upperRPM, double lowerRPM) {
    // Convert RPM (input parameter) to RPS (what Phoenix 6 uses)
    this.upperTargetRPS = upperRPM / 60.0;
    this.lowerTargetRPS = lowerRPM / 60.0;
  }

  @Override
  public void runHooper() {
    // Run Motion Magic Velocity control with FOC
    upperRoller.setControl(m_request.withVelocity(upperTargetRPS));
    parrilla.setControl(m_request.withVelocity(lowerTargetRPS));
  }

  @Override
  public void setVoltage(double voltageUpper, double voltagedown) {
    upperRoller.setVoltage(voltageUpper);
    parrilla.setVoltage(voltagedown);
  }

  @Override
  public void stopMotors() {
    upperRoller.setControl(m_stop);
    parrilla.setControl(m_stop);
  }

  /** Upper roller speed in rotations per second. */
  @Override
  public double getHopperVelo() {
    return upperRoller.getVelocity().getValueAsDouble();
  }
}
