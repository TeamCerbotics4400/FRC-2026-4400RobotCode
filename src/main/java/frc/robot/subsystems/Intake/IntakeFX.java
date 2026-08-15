package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

/** IntakeIO implementation for the real robot: three TalonFX motors and one limit switch. */
public class IntakeFX implements IntakeIO {
  // Roller lider
  private final TalonFX intakeLeft = new TalonFX(IntakeConstants.intakeLeftMotorID);
  private final TalonFXConfiguration intakeLeftConfig = new TalonFXConfiguration();

  private final TalonFX intakeRight = new TalonFX(IntakeConstants.intakeRightMotorID);
  private final TalonFXConfiguration intakeRightConfig = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Pivot
  private final TalonFX pivot = new TalonFX(IntakeConstants.pivotMotorID, "canivore");
  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  /** Conversion from pivot motor rotations to meters of mechanism travel. */
  private double meters_PerRotation = 0.02176272;

  /** Limit switch on DIO 1 that marks the fully retracted position. */
  DigitalInput m_intakelimitSwitch = new DigitalInput(1);

  public IntakeFX() {
    // Intake Config
    intakeLeftConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeLeft.getConfigurator().apply(intakeLeftConfig);

    intakeRightConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeRight.getConfigurator().apply(intakeRightConfig);

    intakeLeftConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeRightConfig.CurrentLimits.SupplyCurrentLimit = 60;

    intakeLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Opposite inversions so both rollers pull a ball in with the same sign of voltage.
    intakeRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeLeft.setNeutralMode(NeutralModeValue.Coast);
    intakeRight.setNeutralMode(NeutralModeValue.Coast);

    // Pivot Config

    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 70;
    pivot.setNeutralMode(NeutralModeValue.Coast);
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Assumes the intake starts retracted at boot.
    pivot.setPosition(0);

    pivot.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void setIntakeLeftVoltage(double volts) {
    intakeLeft.setControl(new VoltageOut(volts));
  }

  @Override
  public void setIntakeRightVoltage(double volts) {
    intakeRight.setControl(new VoltageOut(volts));
  }

  @Override
  public void resetPivotPosition() {
    pivot.setPosition(0);
  }

  @Override
  public void StopIntake() {
    intakeLeft.setControl(voltageRequest.withOutput(0));
    intakeRight.setControl(voltageRequest.withOutput(0));
    pivot.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public double getIntakeVoltage() {
    return intakeLeft.getMotorVoltage().getValueAsDouble();
  }

  /** Pivot position in meters of travel. */
  @Override
  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble() * meters_PerRotation;
  }

  /** Applies voltage to the pivot. The feedforward argument is unused in this implementation. */
  @Override
  public void setVoltage(double volts, double feedforward) {
    pivot.setControl(voltageRequest.withOutput(volts).withEnableFOC(false));
  }

  @Override
  public double getPivotVoltage() {
    return pivot.getMotorVoltage().getValueAsDouble();
  }

  /** The switch reads normally closed, so the signal is inverted. */
  @Override
  public boolean isAtZero() {
    return !m_intakelimitSwitch.get();
  }

  @Override
  public double getCurrentIndexer() {
    return pivot.getStatorCurrent().getValueAsDouble();
  }

  /** Reapplies the whole pivot config with a new stator current limit. */
  @Override
  public void chagePivotCurrent(double current) {
    pivotConfig.CurrentLimits.StatorCurrentLimit = current;
    pivot.getConfigurator().apply(pivotConfig);
  }

  @Override
  public double getCurrentPivot() {
    return pivot.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void updateInputs(IntakeInputs io) {
    io.intakeLeftVoltage = getIntakeVoltage();
    io.intakeRightVoltage = getIntakeVoltage();
    io.pivotPosition = getPivotPosition();
    io.pivotVoltage = getPivotVoltage();
    io.isAtZeroPivot = isAtZero();
    io.currentIndexer = getCurrentIndexer();
    io.pivotCurrent = getCurrentPivot();
  }
}
