package frc.robot.subsystems.Hooper;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the hopper. Real hardware is implemented by HooperFX. */
public interface HooperIO {

  /** Sensor values read once per loop and written to the log by AdvantageKit. */
  @AutoLog
  public class HooperRPMSInputs {
    public double upperRollerRPMs = 0.0;

    /** Speed of the "parrilla" roller, the grid that feeds the shooter. */
    public double parrillaRollerRPMs = 0.0;

    public double upperTargetRPM = 0.0;
    public double parrilaTargetRPM = 0.0;

    public double upperAppliedVolts = 0.0;
    public double parrilaAppliedVolts = 0.0;

    /** Stator current, the main signal used to detect a jam. */
    public double upperStatorCurrent = 0.0;

    public double parrillaStatorCurrent = 0.0;
  }

  default void updateInputs(HooperRPMSInputs inputs) {}

  /** Sets the target speeds in RPM. */
  default void setTargets(double upper, double lower) {}

  /** Runs Motion Magic velocity control toward the stored targets. */
  default void runHooper() {}

  default void stopMotors() {}

  /** Open loop voltage control for both rollers. */
  public default void setVoltage(double voltageUpper, double voltagedown) {}

  public default double getHopperVelo() {
    return 0.0;
  }
}
