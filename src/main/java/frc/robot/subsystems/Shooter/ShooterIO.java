package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * ShooterIO
 *
 * <p>Hardware abstraction for: • Flywheel • Hood angle mechanism
 *
 * <p>Implementations: ShooterFX → real motors ShooterSim → physics simulation
 */
public interface ShooterIO {

  /** Sensor values read once per loop and written to the log by AdvantageKit. */
  @AutoLog
  public static class ShooterInputs {

    /** Average RPM of the four flywheel motors. */
    public double shooterRPMs = 0.0;

    public double targetRPM = 0.0;

    public double ShooterVoltageUpperRight = 0.0;
    public double ShooterVoltageDownRight = 0.0;

    public double statorCurrentUpperRight = 0.0;
    public double statorCurrenDownRight = 0.0;

    public double ShooterVoltageUpperLeft = 0.0;
    public double ShooterVoltageDownLeft = 0.0;

    public double statorCurrentUpperLeft = 0.0;
    public double statorCurrenDownLeft = 0.0;

    // Per-motor RPM, useful for spotting a dead or slipping flywheel
    public double upRightFWRPM = 0.0;
    public double downRightFWRPM = 0.0;
    public double upLeftFWRPM = 0.0;
    public double downLeftFWRPM = 0.0;

    public Rotation2d hoodAngle = Rotation2d.kZero;
    public Rotation2d targetHoodAngle = Rotation2d.kZero;
    public double hoodVelocityDegPerSec = 0.0;

    public double hoodAngleDegrees = 0.0;
    public double targetHoodAngleDegrees = 0.0;

    public double hoodRotations = 0.0;

    // Flywheel gains currently applied
    public double kv = 0.0;
    public double ks = 0.0;
    public double kp = 0.0;
    public double kd = 0.0;

    // Hood gains currently applied
    public double hKv = 0.0;
    public double hks = 0.0;
    public double hkp = 0.0;
    public double hkd = 0.0;
  }

  default void updateInputs(ShooterInputs inputs) {}

  /** Stores the target RPM for logging, does not command the motors. */
  default void setTargetRPM(double rpm) {}

  /** Runs the flywheels at the stored target RPM. */
  default void runShooter() {}

  /** Open loop voltage control, used by SysId and to coast the flywheels down. */
  default void runVolts(double volts) {}

  default void stopMotor() {}

  /** Closed loop velocity control from a supplier, re-read on every call. */
  default void runShotLOL(Supplier<Double> targetRPMs) {}

  default Rotation2d getHoodAngle() {
    return Rotation2d.kZero;
  }

  /** Moves the hood to an absolute angle in degrees. */
  public default void goToAngle(Supplier<Double> angleDegrees) {}

  public default void goToAngleDash(double angleDegrees) {}

  default double getStator() {
    return 0.0;
  }
}
