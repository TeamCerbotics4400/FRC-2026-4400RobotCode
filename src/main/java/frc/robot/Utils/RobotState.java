package frc.robot.Utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * ============================================================ RobotState (Phoenix 6 version)
 * ============================================================
 *
 * <p>Centralized read-only access to robot state information.
 *
 * <p>IMPORTANT: • Phoenix drivetrain already performs pose estimation. • This class DOES NOT run
 * odometry or estimators. • It ONLY reads drivetrain state and provides math utilities.
 *
 * <p>Responsibilities: - Pose access - Velocity access - Distance calculations - Target angles -
 * Alliance flipping
 *
 * <p>NOT responsible for: - Odometry - Sensor fusion - Vision integration
 *
 * <p>Architecture: Phoenix drivetrain → RobotState → other subsystems
 */
public final class RobotState {

  // ============================================================
  // Singleton
  // ============================================================

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {}

  // ============================================================
  // Drivetrain reference (injected once)
  // ============================================================

  private CommandSwerveDrivetrain drivetrain;

  /** Must be called once from RobotContainer after drivetrain construction. */
  public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /** Fails loudly instead of throwing a NullPointerException if the injection was forgotten. */
  private CommandSwerveDrivetrain dt() {
    if (drivetrain == null) {
      throw new IllegalStateException(
          "RobotState drivetrain not set. Call setDrivetrain() in RobotContainer.");
    }
    return drivetrain;
  }

  // ============================================================
  // Pose getters (FROM PHOENIX)
  // ============================================================

  /** Pose from the Phoenix estimator, already fused with vision measurements. */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return dt().getCurrentPosition();
  }

  public Rotation2d getRotation() {
    return getEstimatedPose().getRotation();
  }

  public Translation2d getTranslation() {
    return getEstimatedPose().getTranslation();
  }

  /** True when the heading is within tolerance of targetAngle, handling the 360 degree wrap. */
  public boolean atOrbitAngle(double toleranceDegrees, Rotation2d targetAngle) {
    double currentAngle = drivetrain.getCurrentPosition().getRotation().getDegrees();
    double target = targetAngle.getDegrees();

    // IEEEremainder gives the circular error, so 359 to 1 degree is a 2 degree difference
    // instead of 358.
    double error = Math.abs(Math.IEEEremainder(currentAngle - target, 360));

    return error < toleranceDegrees;
  }

  // ============================================================
  // Velocity getters (FROM PHOENIX)
  // ============================================================

  /** Velocity in the robot frame: X is forward, Y is left. */
  @AutoLogOutput(key = "RobotState/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeVelocity() {
    return dt().getCurrentChassisSpeeds();
  }

  /** Same velocity rotated into the field frame, used by the shoot on the move math. */
  @AutoLogOutput(key = "RobotState/FieldRelativeVelocity")
  public ChassisSpeeds getFieldRelativeVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeVelocity(), getRotation());
  }

  // ============================================================
  // Distance helpers
  // ============================================================

  public Distance getDistanceToPoint(Translation2d point) {
    double meters = getTranslation().getDistance(point);
    return Meters.of(meters);
  }

  /** Distance to our own hub, flipped automatically for the current alliance. */
  @AutoLogOutput(key = "RobotState/DistanceToAllianceHub_m")
  public Distance getDistanceToAllianceHub() {
    return getDistanceToPoint(getAllianceHubTarget().toTranslation2d());
  }

  /** Distance to whatever target the robot should be shooting at right now. */
  @AutoLogOutput(key = "RobotState/DistanceToTargetPoint")
  public Distance getDistanceToTarget() {
    return getDistanceToPoint(FieldUtil.getDynamicTarget(getEstimatedPose()));
  }

  @AutoLogOutput(key = "RobotState/DistanceToOpposingHub_m")
  public Distance getDistanceToOpposingHub() {
    return getDistanceToPoint(getOpposingHubTarget().toTranslation2d());
  }

  // ============================================================
  // Target positions (auto alliance flipped)
  // ============================================================

  public Translation3d getAllianceHubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
  }

  public Translation3d getOpposingHubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.oppTopCenterPoint);
  }

  // ============================================================
  // Angle helpers
  // ============================================================

  /** Returns the heading the robot should face to point at the alliance hub. */
  @AutoLogOutput(key = "RobotState/AngleToAllianceHub")
  public Rotation2d getAngleToAllianceHub() {
    Translation2d target = getAllianceHubTarget().toTranslation2d();
    Translation2d delta = target.minus(getTranslation());

    return new Rotation2d(delta.getX(), delta.getY());
  }

  /** Angle to any field point. */
  public Rotation2d getAngleToPoint(Translation2d point) {
    Translation2d delta = point.minus(getTranslation());
    return new Rotation2d(delta.getX(), delta.getY());
  }
}
