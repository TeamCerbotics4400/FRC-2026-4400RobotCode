// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/**
 * Homes the intake pivot: drives it back at a fixed voltage until the limit switch trips, then
 * zeroes the encoder. Needed because the pivot has a relative encoder with no absolute reference.
 */
public class ResetPivot extends Command {
  /** Creates a new ResetPivot. */
  Intake m_Intake;

  public ResetPivot(Intake m_Intake) {
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  /**
   * Turns off the position PID so the pivot can be driven open loop, and raises the current limit.
   */
  @Override
  public void initialize() {
    m_Intake.disablePID();
    m_Intake.changeCurrentPivot(70);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /** Drives the pivot toward the switch with the rollers stopped. */
  @Override
  public void execute() {
    m_Intake.setVoltsPivotVoid(-12);
    m_Intake.setIntakeVoltageVoid(0);
  }

  // Called once the command ends or is interrupted.
  /**
   * Only zeroes the encoder on a normal finish, since an interruption means the switch never hit.
   */
  @Override
  public void end(boolean interrupted) {
    m_Intake.setVoltsPivotVoid(0);
    if (!interrupted) {
      m_Intake.resetPivotPositionVoid();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Intake.isInSensor();
  }
}
