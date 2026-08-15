package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Utils.ShotCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem: four flywheel motors plus the hood. Targets can be fixed values or suppliers
 * fed by ShotCalculator, which are re-evaluated every loop in periodic().
 */
public class Shooter extends SubsystemBase {
  // IO
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  // SysId
  private final SysIdRoutine sysId;

  /** RPM set from the dashboard, used by the manual tuning commands. */
  public double shooterRPMs = 0.0;

  /** Hood angle set from the dashboard, used by the manual tuning commands. */
  public double hoodAngleDeg = 0.0;

  public boolean atSpeed = false;

  // When non-null these are read every loop, so the shot tracks the robot as it moves.
  private Supplier<Double> flywheelSpeedSupplier;
  private Supplier<Double> hoodAngleSupplier;

  /** Live RPM correction added on top of the interpolated value, tuned from the dashboard. */
  public static double offSet = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;

    // For SysId data logging
    SignalLogger.setPath("/media/sda1/");

    SmartDashboard.putNumber("Shooter/DashRPMs", shooterRPMs);
    SmartDashboard.putNumber("Shooter/DashHoodAngle", hoodAngleDeg);

    SmartDashboard.putNumber("Shooter/offSetDash", offSet);

    // Characterization routine used to find the flywheel feedforward gains.
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.runVolts(volts.in(Volts)), null, this));
  }

  // Shooter commands

  /** Holds the flywheels at a fixed RPM. */
  public Command setRPMCommand(double rpm) {
    return run(
        () -> {
          io.setTargetRPM(rpm);
          io.runShotLOL(() -> rpm);
        });
  }

  public void moveShooterWithRPM() {
    io.setTargetRPM(shooterRPMs);
    io.runShotLOL(() -> shooterRPMs);
  }

  /** Holds the flywheels at the RPM currently typed on the dashboard. */
  public Command setRPMCommandDash() {
    return run(
        () -> {
          io.setTargetRPM(shooterRPMs);
          io.runShotLOL(() -> shooterRPMs);
        });
  }

  public void setRPMVoidDash() {
    io.setTargetRPM(shooterRPMs);
    io.runShotLOL(() -> shooterRPMs);
  }

  public Command stopCommand() {
    return run(io::stopMotor);
  }

  public void stopVoid() {
    io.stopMotor();
  }

  /** Rough check that the flywheels are spun up. */
  public boolean isShooting() {
    return inputs.shooterRPMs > 1500;
  }

  /** Switches the shooter to the distance-based tables so it tracks the target while driving. */
  public Command shootAndMoveLoL() {
    return runOnce(
        () -> {
          flywheelSpeedSupplier =
              () -> ShotCalculator.getInstance().getParameters().flywheelSpeed();

          hoodAngleSupplier = () -> ShotCalculator.getInstance().getParameters().hoodAngle();
        });
  }

  /** Same as shootAndMoveLoL but callable inside another command, and adds the dashboard offset. */
  public void shootAndMoveLoLVoid() {
    flywheelSpeedSupplier =
        () -> ShotCalculator.getInstance().getParameters().flywheelSpeed() + offSet;

    hoodAngleSupplier = () -> ShotCalculator.getInstance().getParameters().hoodAngle();
  }

  public Command goToAngleDashCommand() {
    return this.run(
        () -> {
          io.goToAngle(() -> hoodAngleDeg);
        });
  }

  /** Clears the suppliers, parks the hood at 20 degrees and stops the flywheels. */
  public Command safeHood() {
    return run(
        () -> {
          flywheelSpeedSupplier = null;
          hoodAngleSupplier = null;

          io.goToAngle(() -> 20.0);
          io.runVolts(0);
        });
  }

  /** Reads sensors, refreshes dashboard tuning values and applies the active suppliers. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    offSet = SmartDashboard.getNumber("Shooter/offSetDash", offSet);
    shooterRPMs = SmartDashboard.getNumber("Shooter/DashRPMs", shooterRPMs);
    hoodAngleDeg = SmartDashboard.getNumber("Shooter/DashHoodAngle", hoodAngleDeg);

    SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed(20));
    SmartDashboard.putNumber("Shooter/Target", inputs.targetRPM);
    SmartDashboard.putNumber(
        "Shooter/InterpolatedRPMs", ShotCalculator.getInstance().getParameters().flywheelSpeed());
    SmartDashboard.putNumber(
        "Shooter/InterpolatedAngle", ShotCalculator.getInstance().getParameters().hoodAngle());

    // Re-evaluated every loop so the setpoint follows the robot pose.
    if (flywheelSpeedSupplier != null) {
      double rpm = flywheelSpeedSupplier.get();
      io.setTargetRPM(rpm);
      io.runShotLOL(() -> rpm);
    }

    if (hoodAngleSupplier != null) {
      double angle = hoodAngleSupplier.get();
      io.goToAngle(() -> angle);
    }
  }

  // =========================================================
  // Useful getters (optional)
  // =========================================================

  public double getRPM() {
    return inputs.shooterRPMs;
  }

  public Rotation2d getHoodAngle() {
    return inputs.hoodAngle;
  }

  /** True when the measured flywheel RPM is within toleranceRPM of the target. */
  public boolean atSpeed(double toleranceRPM) {
    return Math.abs(inputs.targetRPM - inputs.shooterRPMs) < toleranceRPM;
  }

  /** True when the measured hood angle is within toleranceAngleDeg of the target. */
  public boolean atAngle(double toleranceAngleDeg) {
    return Math.abs(inputs.targetHoodAngle.getDegrees() - getHoodAngle().getDegrees())
        < toleranceAngleDeg;
  }
}
