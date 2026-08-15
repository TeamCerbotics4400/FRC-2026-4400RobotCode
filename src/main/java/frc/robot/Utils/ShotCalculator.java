package frc.robot.Utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * Turns the robot pose and velocity into a complete shot: heading, hood angle and flywheel RPM.
 *
 * <p>It uses shoot on the move: instead of aiming at the target, it aims at where the target will
 * be relative to the robot after the ball's time of flight, so the shot stays accurate while
 * driving. Values come from interpolation tables measured on the field.
 *
 * <p>Results are cached until clearShootingParameters() is called, so several subsystems can ask
 * for the parameters in the same loop without recomputing them.
 */
public class ShotCalculator {
  private static ShotCalculator instance;

  // Moving average over 5 samples (0.1 s at a 20 ms loop) to smooth the computed velocities.
  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  // Previous values, used to differentiate angle into velocity
  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  /**
   * A complete shot solution. isValid is false when the target is out of the calibrated distance
   * range, which the commands use to hold fire.
   */
  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  /** Cached result for the current loop, cleared by clearShootingParameters(). */
  private ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;

  /** Seconds of latency compensated by projecting the pose forward before aiming. */
  private static double phaseDelay;

  // Interpolation tables measured on the field: key is distance in meters.
  public static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  // Separate tables for long feed shots across the field
  public static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMapFar =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMapFar =
      new InterpolatingDoubleTreeMap();

  /** Time of flight in seconds per distance, used for the shoot on the move correction. */
  public static final InterpolatingDoubleTreeMap shotTOFMap = new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.4;
    maxDistance = 15.0;
    phaseDelay = 0.03;
    /* Score hub */

    shotHoodAngleMap.put(1.8, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.1, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.3, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.85, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(3.1, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(3.4, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(3.8, Rotation2d.fromDegrees(22));
    shotHoodAngleMap.put(4.1, Rotation2d.fromDegrees(24));
    shotHoodAngleMap.put(4.5, Rotation2d.fromDegrees(24.5));
    shotHoodAngleMap.put(4.8, Rotation2d.fromDegrees(32.5));

    shotFlywheelSpeedMap.put(1.9, 1700.0);
    shotFlywheelSpeedMap.put(2.1, 1800.0);
    shotFlywheelSpeedMap.put(2.26, 1890.0);
    shotFlywheelSpeedMap.put(2.55, 1840.0);
    shotFlywheelSpeedMap.put(2.85, 1925.0);
    shotFlywheelSpeedMap.put(3.15, 2000.0);
    shotFlywheelSpeedMap.put(3.5, 2100.0);
    shotFlywheelSpeedMap.put(3.8, 2150.0);
    shotFlywheelSpeedMap.put(4.1, 2200.0);
    shotFlywheelSpeedMap.put(4.5, 2300.0);
    shotFlywheelSpeedMap.put(4.8, 2350.0);

    /* Feed */

    shotHoodAngleMapFar.put(6.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(7.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(8.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(9.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(11.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(13.0, Rotation2d.fromDegrees(40));

    shotFlywheelSpeedMapFar.put(6.0, 2000.0);
    shotFlywheelSpeedMapFar.put(7.0, 2300.0);
    shotFlywheelSpeedMapFar.put(8.0, 2300.0);
    shotFlywheelSpeedMapFar.put(9.0, 2400.0);
    shotFlywheelSpeedMapFar.put(11.0, 2700.0);
    shotFlywheelSpeedMapFar.put(13.0, 2900.0);

    shotTOFMap.put(1.8, 0.96);
    shotTOFMap.put(2.0, .97);
    shotTOFMap.put(2.3, 1.07);
    shotTOFMap.put(2.5, 1.1);
    shotTOFMap.put(2.8, 1.11);
    shotTOFMap.put(3.05, 1.14);
    shotTOFMap.put(3.3, 1.16);
    shotTOFMap.put(3.5, 1.11);
    shotTOFMap.put(3.8, 1.17);
    shotTOFMap.put(4.0, 1.12);
    shotTOFMap.put(4.2, 1.1);
  }

  /** Computes the shot solution, or returns the cached one if it was already computed this loop. */
  public ShootingParameters getParameters() {
    if (latestParameters != null) return latestParameters;

    RobotState state = RobotState.getInstance();

    Pose2d estimatedPose = state.getEstimatedPose();
    ChassisSpeeds robotVel = state.getRobotRelativeVelocity();

    // Project the pose forward by the loop latency so the aim is not one cycle behind.
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotVel.vxMetersPerSecond * phaseDelay,
                robotVel.vyMetersPerSecond * phaseDelay,
                robotVel.omegaRadiansPerSecond * phaseDelay));

    Translation2d target = FieldUtil.getDynamicTarget(estimatedPose);
    Translation2d shooterPos = estimatedPose.getTranslation();
    double distance = shooterPos.getDistance(target);

    ChassisSpeeds fieldVel = state.getFieldRelativeVelocity();
    double vx = fieldVel.vxMetersPerSecond;
    double vy = fieldVel.vyMetersPerSecond;

    // Near means inside our own scoring zone, which selects the short range tables.
    boolean isNear;
    double x = estimatedPose.getX();

    if (!Robot.isRedAlliance()) {
      isNear = x < 3.8;
    } else {
      isNear = x > 12.0;
    }

    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = distance;

    // Fixed point iteration: the time of flight depends on distance and distance depends on the
    // offset caused by that time of flight, so it is solved by iterating until it settles.
    for (int i = 0; i < 20; i++) {
      double tof = calculateDynamicToF(lookaheadDistance, isNear);

      Translation2d offset = new Translation2d(vx * tof, vy * tof);

      lookaheadPose = new Pose2d(shooterPos.plus(offset), estimatedPose.getRotation());

      lookaheadDistance = lookaheadPose.getTranslation().getDistance(target);
    }

    // Aim at the target from the virtual position, which cancels out the robot's own motion.
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();

    Rotation2d hoodRot =
        isNear
            ? shotHoodAngleMap.get(lookaheadDistance)
            : shotHoodAngleMapFar.get(lookaheadDistance);

    hoodAngle = hoodRot.getDegrees();

    double flywheel =
        isNear
            ? shotFlywheelSpeedMap.get(lookaheadDistance)
            : shotFlywheelSpeedMapFar.get(lookaheadDistance);

    // First loop: seed the previous values so the derivative does not spike.
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    // Rate of change of the setpoints, fed forward so the controllers keep up with a moving target.
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);

    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new ShootingParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheel);

    Logger.recordOutput("ShotCalculator/isNear", isNear);
    Logger.recordOutput("ShotCalculator/Distance", lookaheadDistance);

    return latestParameters;
  }

  /** Ball time of flight for a distance. Read from the measured table, not from a physics model. */
  private double calculateDynamicToF(double distance, boolean isNear) {
    /*
    //For sim calculation
        double rpm =
            isNear ? shotFlywheelSpeedMap.get(distance) : shotFlywheelSpeedMapFar.get(distance);

        double angleRad =
            isNear
                ? shotHoodAngleMap.get(distance).getRadians()
                : shotHoodAngleMapFar.get(distance).getRadians();

        double velocityMps =
            (rpm / 60.0) * 2.0 * Math.PI * ShooterConstants.VISUALIZATION_FLYWHEEL_RADIUS;

        double vz = velocityMps * Math.sin(angleRad);

        double dz = FieldConstants.Hub.height - ShooterConstants.SHOOTER_HEIGHT_FROM_GROUND.in(Meters);

        double discriminant = (vz * vz) - (2 * 9.81 * dz);

        if (discriminant < 0) return 0.5;

        // return (vz + Math.sqrt(discriminant)) / 9.81;*/

    return shotTOFMap.get(distance);
  }

  /** Invalidates the cache so the next getParameters() recomputes with a fresh pose. */
  public void clearShootingParameters() {
    latestParameters = null;
  }

  public static double getTOF(double distance) {
    return shotTOFMap.get(distance);
  }
}
