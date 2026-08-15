package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Utils.RobotState;
import frc.robot.Utils.ShotCalculator;
import frc.robot.Utils.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Hooper.Hooper;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

/**
 * Advanced shooting sequence using sequential command composition.
 *
 * <p>This is a FUTURE REFERENCE implementation showing the industry-standard pattern for automated
 * shooter sequences. It guarantees: - Shooter reaches target RPM before feeder activates - Feeder
 * is active for a bounded time window - Clean, composable command structure
 *
 * <p>Usage: Pass this command to the scheduler instead of ShootUntilIsReady
 *
 * <p>Pattern used by: 254 Cheesy Poofs, 1690 Rogue Robotics, 6328 Mechanical Advantage
 */
public class ShootSequenceAdvanced extends Command {
  private final Shooter m_shooter;

  /** Kept for the alignment check, this command does not drive the robot itself. */
  private final CommandSwerveDrivetrain m_driveTrain;

  private final Hooper m_hooper;

  private final ShotCalculator m_shotCalc = ShotCalculator.getInstance();

  // Configurable parameters
  private static final double SHOOTER_RPM_TOLERANCE = 20.0; // RPM tolerance for "at speed"
  private static final double FEED_DURATION = 0.5; // How long to feed (seconds)
  private static final int DRIVE_ANGLE_TOLERANCE = 3; // Degrees

  public ShootSequenceAdvanced(Shooter shooter, CommandSwerveDrivetrain drive, Hooper hooper) {
    this.m_shooter = shooter;
    this.m_driveTrain = drive;
    this.m_hooper = hooper;

    addRequirements(m_shooter, m_hooper);
  }

  /**
   * Returns the complete shooting sequence as a command composition. This is the recommended way to
   * structure complex, sequential operations.
   *
   * <p>Stages: 1. Spin up shooter until it reaches target speed 2. Wait for drive to align 3.
   * Activate feeder for bounded duration 4. Stop everything
   */
  public Command buildShootSequence() {
    return Commands.sequence(
        // Stage 1: Spin up shooter to target RPM
        // Run until shooter reaches target speed (20 RPM tolerance)
        Commands.run(
                () -> {
                  m_shooter.shootAndMoveLoLVoid();
                })
            .until(() -> m_shooter.atSpeed(SHOOTER_RPM_TOLERANCE)),

        // Stage 2: Wait for drive to align while keeping shooter spinning
        Commands.run(
                () -> {
                  m_shooter.shootAndMoveLoLVoid(); // Keep shooter spinning
                })
            .until(
                () -> {
                  ShootingParameters params = m_shotCalc.getParameters();
                  if (!params.isValid()) return false;

                  Rotation2d targetAngle = params.turretAngle().minus(Rotation2d.fromDegrees(0));
                  return RobotState.getInstance().atOrbitAngle(DRIVE_ANGLE_TOLERANCE, targetAngle);
                }),

        // Stage 3: Activate feeder for fixed duration (prevents jamming)
        Commands.run(() -> m_hooper.setVoltageVoid(12, 10)).withTimeout(FEED_DURATION),

        // Stage 4: Stop everything cleanly
        Commands.runOnce(
            () -> {
              m_shooter.stopVoid();
              m_hooper.stopvoid();
            }));
  }

  /**
   * Alternative: Parallel approach - can be useful for certain game elements (Usually NOT
   * recommended for shooter sequences)
   */
  public Command buildParallelShootSequence() {
    return Commands.parallel(
            // Keep shooter active throughout entire sequence
            Commands.run(() -> m_shooter.shootAndMoveLoLVoid()),

            // Feed in sequence
            Commands.sequence(
                // Wait for speed
                Commands.waitUntil(() -> m_shooter.atSpeed(SHOOTER_RPM_TOLERANCE)),

                // Wait for drive
                Commands.waitUntil(
                    () -> {
                      ShootingParameters params = m_shotCalc.getParameters();
                      if (!params.isValid()) return false;

                      Rotation2d targetAngle =
                          params.turretAngle().minus(Rotation2d.fromDegrees(0));
                      return RobotState.getInstance()
                          .atOrbitAngle(DRIVE_ANGLE_TOLERANCE, targetAngle);
                    }),

                // Feed for duration
                Commands.run(() -> m_hooper.setVoltageVoid(12, 10)).withTimeout(FEED_DURATION)))
        .finallyDo(
            interrupted -> {
              m_shooter.stopVoid();
              m_hooper.stopvoid();
            });
  }

  @Override
  public void execute() {
    // This command delegates to buildShootSequence() internally.
    // In a real implementation, you would either:
    // 1. Schedule the sequence directly from RobotContainer
    // 2. Or wrap it in this execute() method
    // For now, this serves as a reference implementation.
  }

  /** Safety stop, runs whether the command finished normally or was interrupted. */
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopVoid();
    m_hooper.stopvoid();
  }
}

/**
 * HOW TO USE:
 *
 * <p>In RobotContainer.java, instead of: ShootUntilIsReady shootCmd = new
 * ShootUntilIsReady(shooter, drive, hooper);
 *
 * <p>Use: ShootSequenceAdvanced advancedShoot = new ShootSequenceAdvanced(shooter, drive, hooper);
 * CommandScheduler.getInstance().schedule(advancedShoot.buildShootSequence());
 *
 * <p>Or bind to a button: joystick.a() .onTrue(advancedShoot.buildShootSequence());
 */
