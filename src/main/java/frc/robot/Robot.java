// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Dashboard.ShiftTimer;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.RobotState;
import frc.robot.Utils.ShotCalculator;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class. Extends LoggedRobot (AdvantageKit) so every loop is recorded and can be
 * replayed later from a log file.
 */
public class Robot extends LoggedRobot {
  /** Autonomous routine chosen on the dashboard, scheduled in autonomousInit. */
  private Command m_autonomousCommand;

  /** Holds every subsystem, the auto chooser and all button bindings. */
  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  /** Interpolation tables that convert distance to target into flywheel RPM and hood angle. */
  private final ShotCalculator m_calculator = ShotCalculator.getInstance();

  double rotationOutput = 0.0;

  /** Configures the log receivers, builds the RobotContainer and starts the logger. */
  public Robot() {
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("frc-2026-RobotCode", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter(Filesystem.getOperatingDirectory().getPath()));
    }

    m_robotContainer = new RobotContainer();

    // Allows capturing the last seconds of camera video after a match.
    LimelightHelpers.setRewindEnabled("limelight-tags", true);

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

  }

  /** Runs every 20 ms in every mode. Publishes distances and runs the command scheduler. */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber(
        "Distance to Hub", RobotState.getInstance().getDistanceToAllianceHub().in(Meters));

    SmartDashboard.putNumber(
        "Distance to target", RobotState.getInstance().getDistanceToTarget().in(Meters));

    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    ShiftTimer.updateDashboard();
  }

  @Override
  public void disabledInit() {

    // LimelightHelpers.triggerRewindCapture("limelight-tags", 150);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** Reads the selected auto routine and schedules it. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Invalidates the cached shot parameters so they are recomputed each loop. */
  @Override
  public void autonomousPeriodic() {
    m_calculator.clearShootingParameters();

    // m_robotContainer.handleAutoAim();
  }

  @Override
  public void autonomousExit() {}

  /** Cancels the autonomous routine so it does not keep running during teleop. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  /** Saves the last 180 frames of Limelight video when teleop ends. */
  @Override
  public void teleopExit() {
    LimelightHelpers.triggerRewindCapture("limelight-tags", 180);
  }

  /** Cancels every running command before entering test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    ShiftTimer.updateDashboard();
  }

  /** Serves the deploy folder over port 5800 (used by Elastic/dashboard layouts). */
  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  /** True when the Driver Station reports the red alliance. Defaults to false if unknown. */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .filter(value -> value == DriverStation.Alliance.Red)
        .isPresent();
  }

  /*
   * Robot coordinate frame:
   *
   * Robot center = (0, 0)
   * +X = forward
   * -X = backward
   * +Y = left
   * -Y = right
   *
   */

  /*
  ███████╗███╗   ███╗ ██████╗      4400
  ██╔════╝████╗ ████║██╔════╝
  ███████╗██╔████╔██║██║  ███╗
  ╚════██║██║╚██╔╝██║██║   ██║
  ███████║██║ ╚═╝ ██║╚██████╔╝
  ╚══════╝╚═╝     ╚═╝ ╚═════╝
  */
}
