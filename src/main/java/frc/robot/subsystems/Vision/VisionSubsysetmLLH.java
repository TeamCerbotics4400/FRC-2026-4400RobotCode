package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionContants;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.PoseEstimate; // Importante
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import limelight.networktables.LimelightSettings.ImuMode;
import org.littletonrobotics.junction.Logger;

/**
 * Alternative vision subsystem built on LimelightHelpers instead of the Limelight vendor library.
 * Same job as VisionSubsystem: feed AprilTag poses into the drivetrain pose estimator.
 */
public class VisionSubsysetmLLH extends SubsystemBase {

  // Camera NetworkTables names

  private final String shooterCam = "limelight-tags";
  private final String lateralCam = "limelight-lateral";

  // Offsets (X, Y, Z, Roll, Pitch, Yaw)
  private final Pose3d shooterOffset =
      new Pose3d(-0.2913309972, 0, 0.34, new Rotation3d(0, -0.26, -Math.PI));

  private final Pose3d lateralOffset =
      new Pose3d(-0.29, 0.34, 0.30, new Rotation3d(0, 0.26, -Math.PI / 2));

  private final CommandSwerveDrivetrain m_drive;

  public VisionSubsysetmLLH(CommandSwerveDrivetrain m_drive) {
    this.m_drive = m_drive;

    // Push the camera mounting offsets on startup
    setCameraOffset(shooterCam, shooterOffset);
    setCameraOffset(lateralCam, lateralOffset);
  }

  /** Sends the camera pose in robot space. Rotations are converted from radians to degrees. */
  private void setCameraOffset(String name, Pose3d offset) {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        offset.getX(),
        offset.getY(),
        offset.getZ(),
        Math.toDegrees(offset.getRotation().getX()),
        Math.toDegrees(offset.getRotation().getY()),
        Math.toDegrees(offset.getRotation().getZ()));
  }

  /** Feeds both cameras the robot orientation and angular rates that MegaTag2 needs. */
  private void updateMT2OrientationNew() {
    // 1. Get the 3D rotation from the swerve drive
    Rotation3d robotRotation = m_drive.getRotation3d();

    // 2. Angular velocities (optional but recommended for accuracy)
    // Read straight from the Pigeon2 exposed by the drivetrain
    double yawRate = m_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    double pitchRate = m_drive.getPigeon2().getAngularVelocityYWorld().getValueAsDouble();
    double rollRate = m_drive.getPigeon2().getAngularVelocityXWorld().getValueAsDouble();

    // 3. Send the full orientation (replaces settings.withRobotOrientation)
    LimelightHelpers.SetRobotOrientation(
        shooterCam,
        robotRotation.getZ() * (180.0 / Math.PI), // Yaw in degrees
        yawRate,
        robotRotation.getY() * (180.0 / Math.PI), // Pitch
        pitchRate,
        robotRotation.getX() * (180.0 / Math.PI), // Roll
        rollRate);

    LimelightHelpers.SetRobotOrientation(
        lateralCam,
        robotRotation.getZ() * (180.0 / Math.PI), // Yaw in degrees
        yawRate,
        robotRotation.getY() * (180.0 / Math.PI), // Pitch
        pitchRate,
        robotRotation.getX() * (180.0 / Math.PI), // Roll
        rollRate);

    // 4. Set the IMU mode dynamically (replaces settings.withImuMode)
    // 0 = Internal IMU, 1 = MT1 Assist (external assistance)
    int imuMode = DriverStation.isEnabled() ? 1 : 0;
    ImuMode imuModeEnum = (imuMode == 1) ? ImuMode.InternalImuMT1Assist : ImuMode.InternalImu;

    // Pipeline 0 is AprilTags, pipeline 1 is the alternative used while disabled
    if (DriverStation.isEnabled()) {
      LimelightHelpers.setPipelineIndex(lateralCam, 1);
      LimelightHelpers.setPipelineIndex(shooterCam, 0);
    } else {
      LimelightHelpers.setPipelineIndex(lateralCam, 1);
      LimelightHelpers.setPipelineIndex(shooterCam, 1);
    }

    // Change the IMU mode through the NetworkTables entry exposed by LimelightHelpers
    LimelightHelpers.getLimelightNTTableEntry(shooterCam, "imumode").setNumber(imuMode);
    LimelightHelpers.getLimelightNTTableEntry(lateralCam, "imumode").setNumber(imuMode);
  }

  @Override
  public void periodic() {

    // 1. Update the orientation for MegaTag2 on both cameras
    updateMT2OrientationNew();

    // 2. Get the best estimate between the two cameras
    PoseEstimate bestEst = getBestEstimate();

    PoseEstimate shooterEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(shooterCam);
    PoseEstimate lateralEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(lateralCam);

    // A pose of exactly (0, 0) means the camera has no valid target
    if (shooterEst.pose.getX() != 0 && shooterEst.pose.getY() != 0) {

      // More tags seen means lower deviation, so the estimator trusts the measurement more
      double xyStdDev =
          VisionContants.xyStdDevCoefficient
              * Math.pow(0.0, 2)
              / ((LimelightHelpers.getTargetCount(shooterCam) == 0)
                  ? 100
                  : LimelightHelpers.getTargetCount(shooterCam));

      m_drive.addVisionMeasurement(shooterEst.pose, shooterEst.timestampSeconds);

      // The huge theta deviation makes the estimator ignore the vision heading and keep the gyro
      m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
    }

    /*if (lateralEst.pose.getX() != 0 && lateralEst.pose.getY() != 0) {

      double xyStdDev = VisionContants.xyStdDevCoefficient
                            * Math.pow(0.0, 2)
            / ((LimelightHelpers.getTargetCount(lateralCam) == 0) ? 100 : LimelightHelpers.getTargetCount(lateralCam));

      m_drive.addVisionMeasurement(lateralEst.pose, lateralEst.timestampSeconds);

      //The nuember 999999 is to not consider the rotation of the cameras
      m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 99999999));
    }*/

    Logger.recordOutput("Vision/LateralPose", lateralEst.pose.getX());
    Logger.recordOutput("Vision/ShooterPose", shooterEst.pose.getX());

    /*  if (bestEst.tagCount > 0)//
         {
          // 3. Compute dynamic standard deviations
          double xyStdDev =
              VisionContants.xyStdDevCoefficient * Math.pow(bestEst.avgTagDist, 2) / bestEst.tagCount;

          // 4. Filter and add to the odometry
          if (isPoseValid(bestEst)) {
            m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
            m_drive.addVisionMeasurement(bestEst.pose, bestEst.timestampSeconds);

            Logger.recordOutput("Vision/BestPose", bestEst.pose);
    }
        }*/
    Logger.recordOutput("Vision/HasUpdate", bestEst != null);
  }

  /** Returns the estimate from whichever camera scores higher, falling back if one sees nothing. */
  private PoseEstimate getBestEstimate() {
    PoseEstimate shooterEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(shooterCam);
    PoseEstimate lateralEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(lateralCam);

    // If one is null or has no data, use the other
    if (shooterEst == null || shooterEst.tagCount == 0) return lateralEst;
    if (lateralEst == null || lateralEst.tagCount == 0) return shooterEst;

    //
    Logger.recordOutput(
        "Vision/ChosedCamera",
        scoreEstimate(shooterEst) >= scoreEstimate(lateralEst)
            ? "shooterEst.pose"
            : "lateralEst.pose");

    // Scoring system with weights
    return (scoreEstimate(shooterEst) >= scoreEstimate(lateralEst)) ? shooterEst : lateralEst;
  }

  /** Confidence score: more tags is better, closer tags are better. */
  private double scoreEstimate(PoseEstimate est) {
    // Weights used to decide which camera is more reliable
    return (est.tagCount * 2.5) + (1.0 / (est.avgTagDist + 0.001));
  }

  /** Rejects poses outside the field, taken while spinning fast, or from tags that are too far. */
  private boolean isPoseValid(PoseEstimate est) {
    // Reject if it falls outside the field boundaries
    boolean outOfBounds =
        (est.pose.getX() < -Constants.FieldConstants.fieldBorderMargin)
            || (est.pose.getX()
                > Constants.FieldConstants.filedLenght + Constants.FieldConstants.fieldBorderMargin)
            || (est.pose.getY() < -Constants.FieldConstants.fieldBorderMargin)
            || (est.pose.getY()
                > Constants.FieldConstants.fieldWidth + Constants.FieldConstants.fieldBorderMargin);

    // Reject if the angular velocity is too high (avoids motion blur)
    boolean spinningTooFast =
        Math.abs(m_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720;

    return !outOfBounds && !spinningTooFast && est.avgTagDist < 6.0;
  }
}
