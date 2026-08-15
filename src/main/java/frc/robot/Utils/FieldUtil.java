package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.Robot;

/**
 * Field zone helpers. Decides what the robot should aim at based on where it is: the hub when
 * inside the scoring zone, or a feed point across the field when outside it. Also holds trench
 * detection zones and a second copy of the shot interpolation tables.
 */
public final class FieldUtil {

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  // Short range tables, used inside the scoring zone
  public static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  // Long range tables, used to feed balls across the field
  public static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMapFar =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMapFar =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.4;
    maxDistance = 15.0;
    phaseDelay = 0.03;
    shotHoodAngleMap.put(1.8, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.1, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.3, Rotation2d.fromDegrees(20));
    shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(24));
    shotHoodAngleMap.put(2.85, Rotation2d.fromDegrees(26));
    shotHoodAngleMap.put(3.1, Rotation2d.fromDegrees(28));
    shotHoodAngleMap.put(3.4, Rotation2d.fromDegrees(30));
    shotHoodAngleMap.put(3.7, Rotation2d.fromDegrees(32));
    shotHoodAngleMap.put(4.0, Rotation2d.fromDegrees(34));
    shotHoodAngleMap.put(4.3, Rotation2d.fromDegrees(34));
    shotHoodAngleMap.put(4.5, Rotation2d.fromDegrees(34));
    shotHoodAngleMap.put(4.75, Rotation2d.fromDegrees(34));

    /* Close range, values raised by 10 */
    shotFlywheelSpeedMap.put(1.9, 1700.0);
    shotFlywheelSpeedMap.put(2.1, 1800.0);
    shotFlywheelSpeedMap.put(2.26, 1900.0);
    shotFlywheelSpeedMap.put(2.55, 1850.0);
    shotFlywheelSpeedMap.put(2.85, 1900.0);
    shotFlywheelSpeedMap.put(3.1, 1950.0);
    shotFlywheelSpeedMap.put(3.4, 2000.0);
    shotFlywheelSpeedMap.put(3.7, 2050.0);
    shotFlywheelSpeedMap.put(4.0, 2100.0);
    shotFlywheelSpeedMap.put(4.3, 2125.0);
    shotFlywheelSpeedMap.put(4.5, 2150.0);
    shotFlywheelSpeedMap.put(4.75, 2190.0);

    /* Feed */

    shotHoodAngleMapFar.put(6.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(7.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(8.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(9.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(11.0, Rotation2d.fromDegrees(40));
    shotHoodAngleMapFar.put(13.0, Rotation2d.fromDegrees(40));

    /* Close range, values raised by 10 */
    shotFlywheelSpeedMapFar.put(6.0, 2000.0);
    shotFlywheelSpeedMapFar.put(7.0, 2300.0);
    shotFlywheelSpeedMapFar.put(8.0, 2300.0);
    shotFlywheelSpeedMapFar.put(9.0, 2400.0);
    shotFlywheelSpeedMapFar.put(11.0, 2700.0);
    shotFlywheelSpeedMapFar.put(13.0, 2900.0);
  }

  private static final double FIELD_LENGTH_METERS = 16.54;

  // HOOD
  // X window where the hood could hit the trench structure

  private static final double BLUE_X_MIN_HOOD = 3.9; // Entry margin
  private static final double BLUE_X_MAX_HOOD = 5.1; // Exit margin

  // X range for the red side (mirrored: 16.54 - 5.1 and 16.54 - 3.9)
  private static final double RED_X_MIN_HOOD = 11.4;
  private static final double RED_X_MAX_HOOD = 12.6;

  // Y thresholds tuned for the side corridors
  private static final double TOP_TRENCH_Y_MIN_HOOD = 6.8; // For Y = 7.34 m
  private static final double BOTTOM_TRENCH_Y_MAX_HOOD = 1.3; // For Y = 0.63 m

  /** True when the robot is where raising the hood risks hitting the trench. */
  public static boolean isInAnyTrenchHOOD(Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    boolean inBlueX = (x >= BLUE_X_MIN_HOOD && x <= BLUE_X_MAX_HOOD);
    boolean inRedX = (x >= RED_X_MIN_HOOD && x <= RED_X_MAX_HOOD);
    boolean inTopTrench = (y >= TOP_TRENCH_Y_MIN_HOOD);
    boolean inBottomTrench = (y <= BOTTOM_TRENCH_Y_MAX_HOOD);

    return (inBlueX || inRedX) && (inTopTrench || inBottomTrench);
  }

  // === TRENCH INTAKE COORDINATES (based on field captures) ===
  // X range for the blue side: about 3.4 m to 6.0 m
  private static final double BLUE_X_MIN_INTAKE = 3.3; // Small safety margin
  private static final double BLUE_X_MAX_INTAKE = 6.1;

  // X range for the red side: mirror of the blue one (16.54 - 6.1 = 10.44 and 16.54 - 3.3
  // = 13.24)
  private static final double RED_X_MIN_INTAKE = 10.4;
  private static final double RED_X_MAX_INTAKE = 13.3;

  // Y thresholds (top and bottom of the field)
  private static final double TOP_TRENCH_Y_MIN_INTAKE = 6.5; // Near 7.39 m
  private static final double BOTTOM_TRENCH_Y_MAX_INTAKE = 1.5; // Near 0.64 m

  /** True when the robot is inside any of the four trenches on the field. */
  public static boolean isInAnyTrenchINTAKE(Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();

    // Check whether we are in the X range of the blue or red trenches
    boolean inBlueX = (x >= BLUE_X_MIN_INTAKE && x <= BLUE_X_MAX_INTAKE);
    boolean inRedX = (x >= RED_X_MIN_INTAKE && x <= RED_X_MAX_INTAKE);

    // Check whether we are at the top or bottom of the Y axis
    boolean inTopTrench = (y >= TOP_TRENCH_Y_MIN_INTAKE);
    boolean inBottomTrench = (y <= BOTTOM_TRENCH_Y_MAX_INTAKE);

    // Inside a trench X range AND in one of the corridors means we are in a trench
    return (inBlueX || inRedX) && (inTopTrench || inBottomTrench);
  }

  // ===== MANUALLY DEFINED TARGETS =====
  // Measured on the field, they do not come from the AprilTag layout.
  private static final Translation2d HUB_BLUE = new Translation2d(4.8, 4.0);
  private static final Translation2d HUB_RED = new Translation2d(12, 3.95);

  /* SEEN FROM THE DRIVER STATION */
  private static final Translation2d BUMPRIGHT_RED = new Translation2d(15.7, 7.0);
  private static final Translation2d BUMPLEFT_RED = new Translation2d(15.7, 2);

  private static final Translation2d BUMPRIGHT_BLUE = new Translation2d(1, 1);
  private static final Translation2d BUMPLEFT_BLUE = new Translation2d(1, 6.0);

  public static Translation2d getHUB() {
    return Robot.isRedAlliance() ? HUB_RED : HUB_BLUE;
  }

  public static Translation2d getRightBUMP() {
    return Robot.isRedAlliance() ? BUMPRIGHT_RED : BUMPRIGHT_BLUE;
  }

  public static Translation2d getLeftBUMP() {
    return Robot.isRedAlliance() ? BUMPLEFT_RED : BUMPLEFT_BLUE;
  }

  /**
   * Chooses what to aim at: the hub when inside our scoring zone, otherwise the closest feed point
   * on our own side of the field.
   */
  public static Translation2d getDynamicTarget(Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();
    boolean isRed = Robot.isRedAlliance();

    double lineaMediaY = 4.0;

    if (!isRed) {
      // === BLUE ===

      if (x < 3.8) {
        return HUB_BLUE;
      } else {

        if (y > lineaMediaY) {
          return BUMPLEFT_BLUE;
        } else {
          return BUMPRIGHT_BLUE;
        }
      }

    } else {
      // === RED ===

      // 1. Are we inside the scoring zone?
      if (x > 12.0) {
        return HUB_RED;
      }

      // 2. If not, top or bottom half?
      else {
        if (y > lineaMediaY) {
          return BUMPRIGHT_RED;
        } else {
          return BUMPLEFT_RED;
        }
      }
    }
  }

  /** Flywheel RPM for the current position, picking the short or long range table by zone. */
  public static double getDynamicInterpolation(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    boolean isRed = Robot.isRedAlliance();
    double x = robotPose.getX();

    // 1. Dynamic target
    Translation2d target = getDynamicTarget(robotPose);

    // 2. Real distance
    double distance = robotTranslation.getDistance(target);

    // 3. Clamp into the calibrated range
    distance = Math.max(minDistance, Math.min(maxDistance, distance));

    // 4. Table selection by zone

    if (!isRed) {
      if (x < 3.8) {
        return shotFlywheelSpeedMap.get(distance); // Scoring zone (close)
      } else {
        return shotFlywheelSpeedMapFar.get(distance); // Outside, long shot
      }
    } else {
      if (x > 11.0) {
        return shotFlywheelSpeedMap.get(distance); // Scoring zone (close)
      } else {
        return shotFlywheelSpeedMapFar.get(distance); // Outside, long shot
      }
    }
  }
}
