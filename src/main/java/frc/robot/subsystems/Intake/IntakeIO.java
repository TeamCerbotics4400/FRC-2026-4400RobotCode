package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the intake. Default no-op methods let a simulation or replay
 * implementation override only what it needs. Real hardware is implemented by IntakeFX.
 */
public interface IntakeIO {

  /** Sensor values read once per loop and written to the log by AdvantageKit. */
  @AutoLog
  public class IntakeInputs {
    public double intakeLeftVoltage = 0.0;
    public double intakeRightVoltage = 0.0;

    /** Pivot travel in meters, converted from motor rotations. */
    public double pivotPosition = 0.0;

    public double pivotCurrent = 0.0;
    public double pivotVoltage = 0.0;

    public double currentIndexer = 0.0;

    /** True when the retracted limit switch is pressed. */
    public boolean isAtZeroPivot = false;

    public double kp = 0.0;
    public double ki = 0.0;
    public double kd = 0.0;
  }

  /** Refreshes every field of the inputs object from the hardware. */
  public default void updateInputs(IntakeInputs io) {}

  public default void setIntakeRightVoltage(double volts) {}

  public default void setIntakeLeftVoltage(double volts) {}

  /** Sets rollers and pivot to zero output. */
  public default void StopIntake() {}

  /** Applies voltage to the pivot motor. */
  public default void setVoltage(double volts, double feedforward) {}

  /** Declares the current pivot position as zero. */
  public default void resetPivotPosition() {}

  public default double getPivotVoltage() {
    return 0.0;
  }

  public default double getIntakeVoltage() {
    return 0.0;
  }

  public default double getPivotPosition() {
    return 0.0;
  }

  /** True when the pivot limit switch is pressed. */
  public default boolean isAtZero() {
    return false;
  }

  public default double getCurrentIndexer() {
    return 0.0;
  }

  /** Applies a new stator current limit to the pivot motor. */
  public default void chagePivotCurrent(double current) {}

  public default double getCurrentPivot() {
    return 0.0;
  }
}
