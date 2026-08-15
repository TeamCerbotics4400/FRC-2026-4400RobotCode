package frc.robot.subsystems.Hooper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HooperContants;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper subsystem: the rollers that move balls from the intake up into the shooter. Also holds the
 * jam detection state.
 */
public class Hooper extends SubsystemBase {

  private final HooperIO io;
  private final HooperRPMSInputsAutoLogged inputs = new HooperRPMSInputsAutoLogged();

  /** Filters out momentary current spikes so a short stall is not reported as a jam. */
  private final Debouncer jamDebouncer =
      new Debouncer(HooperContants.JAM_DEBOUNCE_TIME, DebounceType.kRising);

  private boolean m_isJammed = false;

  public Hooper(HooperIO io) {
    this.io = io;
  }

  public boolean isJammed() {
    return m_isJammed;
  }

  /** Runs the hopper closed loop at the given upper and lower roller speeds, in RPM. */
  public Command setHopperRPMsCommand(double upperRPM, double lowerRPM) {
    return run(
        () -> {
          io.setTargets(upperRPM, lowerRPM);
          io.runHooper();
        });
  }

  /** Stores the target speeds without commanding the motors. */
  public void setHopperRPMs(double upper, double lower) {
    io.setTargets(upper, lower);
  }

  public Command stopCommand() {
    return runOnce(io::stopMotors);
  }

  /** Reverses the rollers for a quarter second to free a jammed ball, then stops. */
  public Command unjamCommand() {
    return run(() -> io.setVoltage(-12, -8)).withTimeout(0.25).finallyDo(() -> io.stopMotors());
  }

  /** True when the upper roller reached its target speed within the given tolerance. */
  public boolean isUpperAtSpeed(double tolerance) {
    return Math.abs(inputs.upperRollerRPMs - inputs.upperTargetRPM) < tolerance;
  }

  public double getUpperRPM() {
    return inputs.upperRollerRPMs;
  }

  /** Open loop voltage control for both rollers. Negative values run the hopper backwards. */
  public Command setVoltage(double voltageUpper, double voltagedown) {
    return run(() -> io.setVoltage(voltageUpper, voltagedown));
  }

  /** Non-command version of setVoltage, called from inside other commands. */
  public void setVoltageVoid(double voltageUpper, double voltagedown) {
    io.setVoltage(voltageUpper, voltagedown);
  }

  public HooperRPMSInputsAutoLogged getInputs() {
    return inputs;
  }

  public void stopvoid() {
    io.stopMotors();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Hopper/Upper Roller Velo", io.getHopperVelo());
    // Jam evaluation for the upper roller (CAN ID 40) goes here.

  }
}
