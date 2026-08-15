package frc.robot.subsystems.Intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem: two roller motors plus a pivot that deploys and retracts the mechanism. The
 * pivot is driven by a profiled PID controller running in periodic().
 */
public class Intake extends SubsystemBase {

  IntakeIO io;
  IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  /** Slower motion profile, used for the shorter shake movements. */
  private TrapezoidProfile.Constraints m_slowPprofile =
      new TrapezoidProfile.Constraints(middleVel, middXLR8tion);

  /** Normal motion profile for deploy and retract. */
  private TrapezoidProfile.Constraints m_profile =
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

  private ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_profile);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  /** When false the pivot is left in open loop, so ResetPivot can drive it manually. */
  private boolean enablePID = false;

  public boolean alreadyRetract = true;

  private final IntakeVisualizer measuredVisualizer =
      new IntakeVisualizer("IntakeVisualized", Color.kGreen);

  /** Position the pivot is being commanded to. */
  public enum PivotState {
    RETRACTED,
    DEPLOYED,
  }

  /** Direction the rollers are being run. */
  public enum IntakeState {
    INTAKING,
    OUTTAKING,
    STOPPED,
  }

  public PivotState pivotState = PivotState.RETRACTED;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /** Runs both rollers at the given voltage. */
  public Command setVolts(double volts) {
    return run(
        () -> {
          io.setIntakeLeftVoltage(volts);
          io.setIntakeRightVoltage(volts);
        });
  }

  /** Drives the pivot open loop at the given voltage. */
  public Command setVoltsPivot(double volts) {
    return run(
        () -> {
          io.setVoltage(volts, 0);
        });
  }

  /** Non-command version of setVoltsPivot, called from inside other commands. */
  public void setVoltsPivotVoid(double volts) {
    io.setVoltage(volts, 0);
  }

  public ProfiledPIDController getController() {
    return m_controller;
  }

  /** Zeroes the pivot encoder and stops the motors. */
  public Command resetPivotPosition() {
    return runOnce(
        () -> {
          io.resetPivotPosition();
          io.StopIntake();
        });
  }

  /** Non-command version of resetPivotPosition. */
  public void resetPivotPositionVoid() {
    io.resetPivotPosition();
    io.StopIntake();
  }

  public Command stopIntake() {
    return run(() -> io.StopIntake());
  }

  /** Hands control of the pivot back to open loop. */
  public void disablePID() {
    enablePID = false;
  }

  /** True when the pivot is pressing the retracted limit switch. */
  public boolean isInSensor() {
    return io.isAtZero();
  }

  /**
   * Sets a new pivot goal and roller voltage. Runs once, the PID then keeps tracking the goal in
   * periodic(). The right roller is inverted so both pull the ball in.
   */
  public Command setPivotPosition(double position, PivotState state, double intakeVolts) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              io.chagePivotCurrent(80);
              getController().reset(inputs.pivotPosition);
              m_controller.setGoal(position);
              enablePID = true;
              pivotState = state;

              io.setIntakeLeftVoltage(intakeVolts);
              io.setIntakeRightVoltage(-intakeVolts);
            },
            this);
    return ejecutable;
  }

  /** Same as setPivotPosition but with a lower pivot current limit for deploying. */
  public Command deployIntake(double position, PivotState state, double intakeVolts) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              io.chagePivotCurrent(60);
              getController().reset(inputs.pivotPosition);
              m_controller.setGoal(position);
              enablePID = true;
              pivotState = state;

              io.setIntakeLeftVoltage(intakeVolts);
              io.setIntakeRightVoltage(-intakeVolts);
            },
            this);
    return ejecutable;
  }

  // For resetPivotCommand
  public void setIntakeVoltageVoid(double voltage) {
    io.setIntakeLeftVoltage(voltage);
    io.setIntakeRightVoltage(-voltage);
  }

  /** Deploys, waits a second and pulls back to pos, shaking loose balls stuck in the intake. */
  public Command shakeIntakeCommand(double pos) {
    return Commands.sequence(
        deployIntake(IntakeDeployed, PivotState.DEPLOYED, 12),
        new WaitCommand(1.0),
        setPivotPosition(pos, PivotState.RETRACTED, 12));
  }

  /** Shake variant without the wait, for autonomous where time matters. */
  public Command shakeIntakeCommandAutonomus(double pos) {
    return Commands.sequence(
        deployIntake(IntakeDeployed, PivotState.DEPLOYED, 12),
        setPivotPosition(pos, PivotState.RETRACTED, 0));
  }

  public double getCurrentIndexer() {
    return io.getCurrentIndexer();
  }

  /** Changes the pivot stator current limit at runtime. */
  public void changeCurrentPivot(double current) {
    io.chagePivotCurrent(current);
  }

  /** Reads sensors, runs the pivot closed loop when enabled and logs the state. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (enablePID) {
      io.setVoltage(
          m_controller.calculate(inputs.pivotPosition),
          m_feedforward.calculate(m_controller.getSetpoint().velocity));
    }

    measuredVisualizer.setIntakePosition(inputs.pivotPosition);

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", pivotState);
    Logger.recordOutput("Intake/PivotVoltage", inputs.pivotVoltage);
    Logger.recordOutput("Intake/CurrentIndexer", getCurrentIndexer());

    SmartDashboard.putBoolean("Intake/inSensor", isInSensor());
  }
}
