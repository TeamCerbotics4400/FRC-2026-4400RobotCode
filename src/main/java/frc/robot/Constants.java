package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Central place for CAN IDs, gains, setpoints and field poses used across the project. */
public class Constants {

  /** Skips WPILib HAL calls, used by unit tests running off the robot. */
  public static boolean disableHAL = false;

  /** Mode assumed when the code is not running on real hardware. */
  public static final Mode simMode = Mode.SIM;

  /** Mode actually in use: REAL on the roboRIO, simMode otherwise. */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** When true, LoggedTunableNumber values are read from the dashboard instead of the defaults. */
  public static final boolean tuningMode = true;

  /** Degrees of heading error still accepted as "aimed at the target". */
  public static final double DRIVE_ANGLE_TOLERANCE = 4.0;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Execution environment of the robot code. */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,

    CASCARITO
  }

  /** Hopper (ball feeding path) CAN IDs and jam detection thresholds. */
  public class HooperContants {

    public static final int upperRollerID = 40;
    public static final int lowerRollerID = 42;
    public static final int parrillaID = 41;
    public static final int FLRollerID = 43;

    // Anti-Jam Constants
    public static final double JAM_CURRENT_THRESHOLD = 50.0; // Amps
    public static final double JAM_VELOCITY_THRESHOLD = 1000.0; // RPM (~25 RPS)
    public static final double JAM_VOLTAGE_THRESHOLD = 2.0; // Volts
    public static final double JAM_DEBOUNCE_TIME = 0.1; // Seconds
  }

  /** Flywheel velocity gains and shooter motor CAN IDs. */
  public class ShooterConstants {

    public static double ShooterKP = 55.0;
    public static double ShooterKS = 0.3;
    public static double ShooterKV = 0.5;
    public static double ShooterKA = 0.065;

    /* Can IDs */
    public static final int flywheeUPlMotorIDRight = 30;
    public static final int flywheeDownMotorIDRight = 31;

    public static final int flywheeUPlMotorIDLeft = 32;
    public static final int flywheeDownMotorIDLeft = 33;
  }

  /** Scales how much the pose estimator trusts a vision measurement. Higher means less trust. */
  public static class VisionContants {
    public static double xyStdDevCoefficient = 0.2;
    public static double thetaStdDevCoefficient = 0.4;
  }

  /** Hood motor CAN ID and its gear reduction. */
  public static class HoodConstants {
    public static final int hoodMotorID = 34;

    /** Hood degrees travelled per full rotation of the motor. */
    public static final double kHoodDegreesPerMotorRotation = 5.741626;
  }

  /** Field poses used for auto alignment, per alliance. */
  public class FieldConstants {

    // Path planner for generate a trayectory but isnt to accurate
    public static final Pose2d alignBluePoseTower =

        // Blue tower
        new Pose2d(1.5, 4.0, new Rotation2d(Units.degreesToRadians(180)));

    public static final Pose2d alignRedPoseTower =

        // Red tower
        new Pose2d(14.9, 4.75, new Rotation2d(Units.degreesToRadians(0))); // Der

    // Final poses for auto Align and climb

    public static final Pose2d finalRedPoseTower =
        // Red tower
        new Pose2d(14.92, 4.0, Rotation2d.fromDegrees(0));

    public static final Pose2d finalBluePoseTower =
        // Blue tower
        new Pose2d(1.6, 4.0, Rotation2d.fromDegrees(180));

    /* Auto aligns for shoot */

    // Red Zone align

    // Align to red left bump
    public static final Pose2d redLeftBump = new Pose2d(13.5, 1.1, Rotation2d.fromDegrees(180));

    // Align to red right bump
    public static final Pose2d redRightBump = new Pose2d(13.5, 6.9, Rotation2d.fromDegrees(180));

    // Blue Zone align
    // Align to blue left bump
    public static final Pose2d blueLeftBump = new Pose2d(3, 7.0, Rotation2d.fromDegrees(0));

    // Align to blue right bump
    public static final Pose2d blueRightBump = new Pose2d(3, 1.0, Rotation2d.fromDegrees(0));

    public static final double fieldWidth = 4.0;
    public static final double filedLenght = 17.29;

    /** Extra margin allowed outside the field before a vision pose is rejected. */
    public static final double fieldBorderMargin = 0.01;
  }

  /** Intake pivot setpoints, CAN IDs and motion profile gains. */
  public static class IntakeConstants {

    /** Pivot position (in meters of travel) with the intake fully out. */
    public static double IntakeDeployed = .24; // 0.245

    /** Intermediate pivot position used to shake the intake. */
    public static double MiddleIntake = .18; // 0.18

    public static final int intakeLeftMotorID = 21;
    public static final int intakeRightMotorID = 20;
    public static final int pivotMotorID = 22;

    // Profiled PID and feedforward gains for the pivot
    public static double kP = 100, kI = 0.0, kD = 0.0, kS = 0.0, kV = 200, kA = 0.0;
    public static double maxVelocity = 300.0;
    public static double maxAcceleration = 300.0;

    // Constraints for the slower pivot profile
    public static double middleVel = 50;
    public static double middXLR8tion = 50;
  }
}
