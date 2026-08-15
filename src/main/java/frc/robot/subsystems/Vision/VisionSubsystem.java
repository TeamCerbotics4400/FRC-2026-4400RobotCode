package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.target.pipeline.NeuralClassifier;

// ... (imports stay the same)

/**
 * AprilTag localization using the Limelight vendor library. Reads pose estimates from the cameras,
 * filters them by quality and feeds the good ones into the drivetrain pose estimator.
 */
public class VisionSubsystem extends SubsystemBase {

  /** Position and orientation of the shooter camera relative to the robot center, in meters. */
  Pose3d cameraOffset =
      new Pose3d(
          Meters.of(-0.2913309972).in(Meters),
          Meters.of(0).in(Meters),
          Meters.of(0.34).in(Meters),
          new Rotation3d(0, -0.26, Math.PI));

  /* Lateral */
  /** Same offset for the side mounted camera, rotated 90 degrees. */
  Pose3d cameraOffsetLateral =
      new Pose3d(
          Meters.of(0.36).in(Meters),
          Meters.of(-0.23).in(Meters),
          Meters.of(0.32).in(Meters),
          new Rotation3d(0, 0.26, 3.14159 / 2));

  Limelight limelight;
  Limelight lateral;

  // MegaTag1 works without a gyro heading, MegaTag2 needs one but is more accurate while moving.
  LimelightPoseEstimator poseEstimatorM2;
  LimelightPoseEstimator poseEstimatorM1;
  LimelightPoseEstimator lateralM1;
  LimelightPoseEstimator lateralM2;

  CommandSwerveDrivetrain m_drive;

  /** Wrapped in try/catch so a disconnected camera does not stop robot code from starting. */
  public VisionSubsystem(CommandSwerveDrivetrain m_drive) {
    this.m_drive = m_drive;

    try {
      limelight = new Limelight("limelight-tags");
      lateral = new Limelight("limelight-lateral");

      poseEstimatorM1 = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
      poseEstimatorM2 = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

      lateralM1 = lateral.createPoseEstimator(EstimationMode.MEGATAG1);
      lateralM2 = lateral.createPoseEstimator(EstimationMode.MEGATAG2);

      setupLimelight(limelight, cameraOffset);
      setupLimelight(lateral, cameraOffsetLateral);

      System.out.println("Initialize limelight");
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  /** MegaTag1 pose, which does not depend on the gyro heading. Empty when no tag is visible. */
  public Optional<Pose2d> getMegaTag1Pose2d() {
    return poseEstimatorM1.getPoseEstimate().map(estimate -> estimate.pose.toPose2d());
  }

  /** Writes the camera mounting offset into the Limelight so it can compute the robot pose. */
  private void setupLimelight(Limelight cam, Pose3d offset) {
    cam.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(offset).save();
  }

  /**
   * Sends the current robot heading to the camera (required by MegaTag2), then accepts the pose
   * estimate only if the tags are close, present and unambiguous.
   */
  private void updateCameraPose(
      Limelight cam, LimelightPoseEstimator m1, LimelightPoseEstimator m2) {
    var settings = cam.getSettings();
    settings.withRobotOrientation(
        new Orientation3d(
            m_drive.getRotation3d(),
            new AngularVelocity3d(
                DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))));

    settings.withImuMode(
        DriverStation.isEnabled() ? ImuMode.InternalImuMT1Assist : ImuMode.InternalImu);
    // When is disable doesnt see the tags//
    settings.withPipelineIndex(DriverStation.isEnabled() ? 0 : 0);
    settings.save();

    // MegaTag2 while enabled, MegaTag1 while disabled so the heading can be seeded from tags.
    LimelightPoseEstimator activeEst = DriverStation.isEnabled() ? m2 : m1;
    activeEst
        .getPoseEstimate()
        .ifPresent(
            estimate -> {
              // Quality gate: far or ambiguous tags produce bad poses and are discarded.
              if (estimate.avgTagDist < 4
                  && estimate.tagCount > 0
                  && estimate.getMinTagAmbiguity() < 0.3) {
                m_drive.addVisionMeasurement(estimate.pose.toPose2d(), estimate.timestampSeconds);
              }
            });
  }

  @Override
  public void periodic() {

    updateCameraPose(limelight, poseEstimatorM1, poseEstimatorM2);
    // updateCameraPose(lateral, lateralM1, lateralM2);

    limelight.getData().getResults().isPresent();

    // Logger.recordOutput("Vision/Detecting", (limelight.getLatestResults() == null) ? 0 :
    // limelight.getLatestResults().get().botpose_tagcount );

    // Neural detector results for game pieces. Hook left in place, no action taken yet.
    limelight
        .getLatestResults()
        .ifPresent(
            (LimelightResults result) -> {
              for (NeuralClassifier object : result.targets_Classifier) {
                // Classifier says its a note.
                if (object.className.equals("fuel")) {
                  if (object.ty > 2 && object.ty < 1) {
                    // do stuff
                  }
                }
              }
            });

    lateral
        .getLatestResults()
        .ifPresent(
            (LimelightResults result) -> {
              for (NeuralClassifier object : result.targets_Classifier) {
                // Classifier says its a note.
                if (object.className.equals("fuel")) {
                  if (object.ty > 2 && object.ty < 1) {
                    // do stuff
                  }
                }
              }
            });

    simulationPeriodic();
  }

  /*  Pose3d cameraOffset =

      new Pose3d(

          Meters.of(-0.31).in(Meters),

          Meters.of(0.28).in(Meters),

          Meters.of(0.46).in(Meters),

          new Rotation3d(0, 0.26, Math.PI));



  Pose3d cameraOffsetLateral =

      new Pose3d(

          Meters.of(0.36).in(Meters),

          Meters.of(-0.23).in(Meters),

          Meters.of(0.32).in(Meters),

          new Rotation3d(0, 0.26, Math.PI / 2));

  Limelight limelight;
  Limelight lateral;

  LimelightPoseEstimator poseEstimatorM1, poseEstimatorM2;
  LimelightPoseEstimator lateralM1, lateralM2;

  CommandSwerveDrivetrain m_drive;

  public VisionSubsystem(CommandSwerveDrivetrain m_drive) {
    this.m_drive = m_drive;

    try {
      limelight = new Limelight("limelight-tags");
      lateral = new Limelight("limelight-lateral");

      poseEstimatorM1 = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
      poseEstimatorM2 = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

      lateralM1 = lateral.createPoseEstimator(EstimationMode.MEGATAG1);
      lateralM2 = lateral.createPoseEstimator(EstimationMode.MEGATAG2);

      setupLimelight(limelight, cameraOffset);
      setupLimelight(lateral, cameraOffsetLateral);

    } catch (Exception e) {
      DriverStation.reportError("Error starting Vision: " + e.getMessage(), true);
    }
  }

  private void setupLimelight(Limelight cam, Pose3d offset) {
    cam.getSettings()
        .withCameraOffset(offset)
        .save();
  }

  // FIX: now returns PoseEstimate and takes the robot angle
  private Optional<PoseEstimate> getEstimateFromCamera(Limelight cam, LimelightPoseEstimator m1, LimelightPoseEstimator m2) {

    // MegaTag2 needs the current robot orientation to be accurate
    //cam.getSettings().withImuMode(null);

    var settings = cam.getSettings();
    settings.withRobotOrientation(
        new Orientation3d(
            m_drive.getRotation3d(),
            new AngularVelocity3d(
                DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))));

    settings.withImuMode(
        DriverStation.isEnabled() ? ImuMode.InternalImuMT1Assist : ImuMode.InternalImu);

    LimelightPoseEstimator activeEstimator = DriverStation.isEnabled() ? m2 : m1;
    Optional<PoseEstimate> poseOpt = activeEstimator.getPoseEstimate();

    // Filter the estimate by quality
    return poseOpt.filter(est ->
        est.hasData &&
        est.tagCount > 0 &&

        est.avgTagDist < 4.0 &&
        est.getMinTagAmbiguity() < 0.3
    );
  }

  // FIX: takes a PoseEstimate, not the estimator
  private double scoreEstimate(PoseEstimate est) {
    double tagWeight = 2.5;
    double distWeight = 1.0;
    double ambiguityWeight = 2.0;

    double tagScore = est.tagCount * tagWeight;
    double distScore = distWeight / (est.avgTagDist + 0.001);
    // Note: lower ambiguity means a better score
    double ambiguityScore = ambiguityWeight / (est.getMinTagAmbiguity() + 0.001);

    return tagScore + distScore + ambiguityScore;
  }

  private Optional<PoseEstimate> getBestEstimate() {
    Optional<PoseEstimate> shooterEst = getEstimateFromCamera(limelight, poseEstimatorM1, poseEstimatorM2);
    Optional<PoseEstimate> lateralEst = getEstimateFromCamera(lateral, lateralM1, lateralM2);

    if (shooterEst.isEmpty()) return lateralEst;
    if (lateralEst.isEmpty()) return shooterEst;

    // If both have data, compare their quality
    return (scoreEstimate(shooterEst.get()) >= scoreEstimate(lateralEst.get())) ? shooterEst : lateralEst;
  }

  private void processDetections(Limelight cam) {
    cam.getLatestResults().ifPresent(result -> {
      for (NeuralClassifier object : result.targets_Classifier) {
        // "note" was the common name in 2024, "fuel" comes from earlier years.
        // Check the object name in your Limelight pipeline.
        if (object.className.equals("fuel")) {
          if (object.ty > 1 && object.ty < 2) {
            // intake or auto-aim logic
          }
        }
      }
    });
  }

  @Override
  public void periodic() {
    Optional<PoseEstimate> best = getBestEstimate();

    best.ifPresent(est -> {
      m_drive.addVisionMeasurement(
          est.pose.toPose2d(),
          est.timestampSeconds
      );
    });

    Logger.recordOutput("Vision/HasBestEstimate", best.isPresent());
    processDetections(limelight);
    processDetections(lateral);
  }*/
}
