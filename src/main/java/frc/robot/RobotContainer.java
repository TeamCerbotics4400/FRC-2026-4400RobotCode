// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.IntakeConstants.IntakeDeployed;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.Intake.ResetPivot;
import frc.robot.Commands.OrbitinWithMovement;
import frc.robot.Commands.Paths.Auto;
import frc.robot.Commands.Paths.LCAutoLeft;
import frc.robot.Commands.Paths.LCAutoRight;
import frc.robot.Commands.Paths.LagunaAuto;
import frc.robot.Commands.Paths.SecondPickAuto;
// import frc.robot.Commands.Paths.SecondPickAuto;
import frc.robot.Commands.ShootUntilIsReady;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Dashboard.ShiftTimer;
import frc.robot.Utils.LocalADStarAK;
import frc.robot.Utils.RobotState;
import frc.robot.subsystems.Hooper.Hooper;
import frc.robot.subsystems.Hooper.HooperFX;
import frc.robot.subsystems.Hooper.HooperIO;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PivotState;
import frc.robot.subsystems.Intake.IntakeFX;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterFX;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.TunerConstants;
import frc.robot.subsystems.Vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Builds every subsystem, registers the autonomous routines and maps the driver controller to
 * commands. Instantiated once from Robot.
 */
public class RobotContainer {

  private final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandSwerveDrivetrain m_drive = TunerConstants.createDrivetrain();

  /** Feeds AprilTag pose estimates into the drivetrain pose estimator. */
  private final VisionSubsystem m_vision = new VisionSubsystem(m_drive);

  /** Publishes swerve state to NetworkTables and the log file. */
  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  /* Chooser for autonomous */
  private final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();

  /* Set up for utils */
  /** Dashboard widget that draws the path of the selected auto routine. */
  public static Field2d autoFieldPreview = new Field2d();

  // IO layers: the ternary lets a simulation implementation be swapped in later.
  private static final IntakeIO intakeIO =
      RobotBase.isReal() ? new IntakeFX() : new IntakeFX(); // new IntakeFX();
  public static Intake m_intake;

  public static final ShooterIO shooterIO = RobotBase.isReal() ? new ShooterFX() : new ShooterFX();
  public static Shooter m_shooter;

  private static final HooperIO hooperIO = new HooperFX();
  public static Hooper m_hooper;

  public boolean isNear;

  private static final ShiftTimer shiftTimer = new ShiftTimer();

  public RobotContainer() {

    m_intake = new Intake(intakeIO);
    m_shooter = new Shooter(shooterIO);
    m_hooper = new Hooper(hooperIO);

    // RobotState reads pose and velocity from the drivetrain, so it needs the reference.
    RobotState.getInstance().setDrivetrain(m_drive);

    enableNamedCommands();

    // Autonomous options shown on the dashboard.
    autoChooser.setDefaultOption(" Optimizes 1 Cicle + Depot", new LagunaAuto());
    autoChooser.addOption("Laguna + 1678 Auto", new LCAutoLeft());
    autoChooser.addOption("Laguna + 1678 Auto Right", new LCAutoRight());
    autoChooser.addOption("SecondPickAuto", new SecondPickAuto());

    autoChooser.addOption("AUTO", new Auto());

    // Redraw the field preview whenever a different auto is selected.
    autoChooser.onChange(
        auto -> {
          if (auto != null) {
            autoFieldPreview.getObject("path").setPoses(auto.getAllPathPoses());
          } else {
            autoFieldPreview.getObject("path").setPoses(new Pose2d[] {});
          }
        });

    // Send the PathPlanner active path and target pose to the log.
    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    // Use the AdvantageKit-compatible pathfinder so pathfinding is replayable.
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Warmup commands run the path code once so the first real path is not delayed by JIT.
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();

    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Auto Preview", autoFieldPreview);

    configureBindings();
  }

  /** Maps controller buttons to commands and sets the default drive command. */
  private void configureBindings() {

    // Drive//
    // Left stick translates, right stick rotates. Axes are negated to match the field frame.
    m_drive.setDefaultCommand(
        new DriveCommand(
            m_drive,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));

    // Right bumper: deploy the intake and run the rollers, retract on release.
    joystick
        .rightBumper()
        .whileTrue(
            m_intake.deployIntake(
                Constants.IntakeConstants.IntakeDeployed, PivotState.DEPLOYED, 12))
        .whileFalse(m_intake.setPivotPosition(IntakeDeployed, PivotState.RETRACTED, 0));

    // Left bumper: aim at the target while driving slowly, shoot once ready and shake the intake.
    joystick
        .leftBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new OrbitinWithMovement(
                        m_drive, () -> joystick.getLeftY() / 7, () -> joystick.getLeftX() / 7),
                    new ShootUntilIsReady(m_shooter, m_hooper),
                    m_intake.shakeIntakeCommand(0.05))))
        .onFalse(
            new ParallelCommandGroup(
                m_hooper.stopCommand(), m_shooter.safeHood(), new ResetPivot(m_intake)));

    // Y: fixed 2000 RPM shot, used when the interpolation tables are not wanted.
    joystick
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                m_shooter.setRPMCommand(2000), m_intake.shakeIntakeCommand(0.05)))
        .onFalse(
            new ParallelCommandGroup(
                m_hooper.stopCommand(), m_shooter.safeHood(), new ResetPivot(m_intake)));

    // Start: zero the gyro heading facing away from the driver station.
    joystick
        .start()
        .onTrue(
            m_drive
                .runOnce(
                    () ->
                        m_drive.resetRotation(new Rotation2d(Robot.isRedAlliance() ? Math.PI : 0)))
                .ignoringDisable(true));

    // B: run intake and hopper in reverse to clear a jam.
    joystick
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                m_intake.setPivotPosition(
                    Constants.IntakeConstants.IntakeDeployed, PivotState.DEPLOYED, -12),
                m_hooper.setVoltage(-12, -12)))
        .whileFalse(
            new ParallelCommandGroup(
                m_intake.setPivotPosition(
                    Constants.IntakeConstants.IntakeDeployed, PivotState.DEPLOYED, 0),
                m_hooper.setVoltage(0, 0)));

    // Left trigger: aim, spin up from the interpolation tables and keep intaking.
    joystick
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                new OrbitinWithMovement(
                    m_drive, () -> joystick.getLeftY() / 7, () -> joystick.getLeftX() / 7),
                smartShootCommand(),
                m_intake.setPivotPosition(
                    Constants.IntakeConstants.IntakeDeployed, PivotState.DEPLOYED, 12)))
        .onFalse(
            new ParallelCommandGroup(
                m_hooper.stopCommand(), m_shooter.safeHood(), new ResetPivot(m_intake)));

    // X: drive the pivot down until the limit switch is hit and re-zero the encoder.
    joystick.x().onTrue(new ResetPivot(m_intake));

    m_drive.registerTelemetry(logger::telemeterize);
  }

  /**
   * Spins up the shooter using the distance-based tables, waits 1 s for it to reach speed, then
   * feeds balls with the hopper.
   */
  public Command smartShootCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              m_shooter.shootAndMoveLoLVoid();
              // m_shooter.setRPMVoidDash();
            },
            m_shooter),
        Commands.waitSeconds(1.0),
        Commands.run(
            () -> {
              m_hooper.setVoltageVoid(12, 12);
            },
            m_hooper));
  }

  /** Same as smartShootCommand but waits longer, used by the second pick auto. */
  public Command smartShootCommandSP() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              m_shooter.shootAndMoveLoLVoid();
              // m_shooter.setRPMVoidDash();
            },
            m_shooter),
        Commands.waitSeconds(1.75),
        Commands.run(
            () -> {
              m_hooper.setVoltageVoid(12, 12);
            },
            m_hooper));
  }

  /** Pathfinds to the shooting pose that matches the current side of the field. */
  public Command pathFindAndAlignCommandToShoot() {
    return m_drive.goToPose(() -> getDynamicAutoAlingn());
  }

  /** Returns the selected auto routine, or a no-op command when nothing is selected. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected() != null ? autoChooser.getSelected() : Commands.none();
  }

  /** Picks the closest alignment pose based on which half of the field the robot is on. */
  public static Pose2d getDynamicAutoAlingn() {
    double y = RobotState.getInstance().getEstimatedPose().getY();
    boolean isRed = Robot.isRedAlliance();
    double lineaMediaY = 4.0;

    if (!isRed) {
      if (y > lineaMediaY) {
        return Constants.FieldConstants.blueLeftBump;
      } else {
        return Constants.FieldConstants.blueRightBump;
      }
    } else {
      if (y < lineaMediaY) {
        return Constants.FieldConstants.redLeftBump;
      } else {
        return Constants.FieldConstants.redRightBump;
      }
    }
  }

  /** True when the driver is moving any stick past the deadband. */
  private boolean isJoystickActive() {
    double deadband = 0.2;
    return Math.abs(joystick.getLeftX()) > deadband
        || Math.abs(joystick.getLeftY()) > deadband
        || Math.abs(joystick.getRightX()) > deadband;
  }

  public static CommandSwerveDrivetrain getDriveSubsystem() {
    return m_drive;
  }

  public static Intake getIntakeSubsystem() {
    return m_intake;
  }

  public static Shooter getShooterSubsystem() {
    return m_shooter;
  }

  public ShiftTimer getShiftTimer() {
    return shiftTimer;
  }

  /** Exposes commands to PathPlanner so they can be triggered from event markers in a path. */
  private void enableNamedCommands() {
    NamedCommands.registerCommand(
        "IntakeCommand",
        m_intake.setPivotPosition(IntakeConstants.IntakeDeployed, PivotState.DEPLOYED, 12));
    NamedCommands.registerCommand(
        "StopIntaking", m_intake.setPivotPosition(IntakeDeployed, PivotState.RETRACTED, 0));
    NamedCommands.registerCommand("StopShooter", m_shooter.safeHood());
    NamedCommands.registerCommand("ShootandMove", m_shooter.shootAndMoveLoL());
    NamedCommands.registerCommand("ActivateHopper", m_hooper.setVoltage(12, 12));
    NamedCommands.registerCommand("DisableHopper", m_hooper.setVoltage(0, 0));
    NamedCommands.registerCommand("SmartShootCommand", smartShootCommand());
    NamedCommands.registerCommand("SmartShootCommand2P", smartShootCommandSP());
    NamedCommands.registerCommand("ShakeIntake", m_intake.shakeIntakeCommand(0.05));
    NamedCommands.registerCommand(
        "AutoAlign", new OrbitinWithMovement(m_drive, () -> 0.0, () -> 0.0));
    NamedCommands.registerCommand("ReverseHopper", m_hooper.setVoltage(-12, -12));
  }

  /** Forwards the simulation tick to the subsystems that model physics. */
  public void simulationPeriodic() {
    m_vision.simulationPeriodic();
    if (RobotBase.isSimulation()) {
      m_shooter.simulationPeriodic();
    }
  }
}
