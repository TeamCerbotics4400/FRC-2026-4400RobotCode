package frc.robot.Dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Tracks the alternating "shift" windows of the match, where only one alliance is allowed to score
 * at a time. Purely informational: it publishes the current window to the dashboard and does not
 * control any mechanism.
 */
public class ShiftTimer {

  /* =========================
  GAME STATE ENUM
  ========================= */
  public enum GameState {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED
  }

  /* =========================
  TIME MARKS (MatchTime)
  ========================= */

  // Match time counts down, so a larger number means earlier in the match.
  private static final double SHIFT1_START = 130; // 2:10
  private static final double SHIFT2_START = 105; // 1:45
  private static final double SHIFT3_START = 80; // 1:20
  private static final double SHIFT4_START = 55; // 0:55
  private static final double ENDGAME_START = 30; // 0:30

  /* =========================
  WHO WON AUTO
  ========================= */

  /** The field reports the auto winner in the first character of the game specific message. */
  public static boolean blueWonAuto() {
    String msg = DriverStation.getGameSpecificMessage();
    if (msg != null && msg.length() > 0) {
      return msg.charAt(0) == 'B';
    }
    return false;
  }

  /* =========================
  WHO STARTS SHIFT1
  (the alliance that wins auto does NOT start)
  ========================= */

  private static boolean doesBlueStartShift1() {
    return !blueWonAuto();
  }

  /* =========================
  ACTIVE FIRST (YES / NO)
  ========================= */

  /** "YES" when our alliance owns the first shift window. */
  public static String getActiveFirst() {

    boolean blueStarts = doesBlueStartShift1();
    boolean weAreBlue = !Robot.isRedAlliance();

    boolean weStart = (blueStarts && weAreBlue) || (!blueStarts && !weAreBlue);

    return weStart ? "YES" : "NO";
  }

  /* =========================
  MATCH TIME
  ========================= */

  /** Seconds remaining in the current period, as reported by the Driver Station. */
  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  /* =========================
  GAME STATE
  ========================= */

  /** Maps the remaining match time to the shift window currently running. */
  public static GameState getCurrentGameState() {

    if (!DriverStation.isEnabled()) return GameState.DISABLED;

    if (DriverStation.isAutonomousEnabled()) return GameState.AUTO;

    double time = getMatchTime();

    if (time > SHIFT1_START) return GameState.TRANSITION;
    if (time > SHIFT2_START) return GameState.SHIFT1;
    if (time > SHIFT3_START) return GameState.SHIFT2;
    if (time > SHIFT4_START) return GameState.SHIFT3;
    if (time > ENDGAME_START) return GameState.SHIFT4;

    return GameState.ENDGAME;
  }

  /* =========================
  REMAINING SHIFT TIME
  ========================= */

  /** Seconds left before the current shift window ends. */
  public static double getRemainingShiftTime() {

    double time = getMatchTime();

    if (time > SHIFT1_START) return time - SHIFT1_START;
    if (time > SHIFT2_START) return time - SHIFT2_START;
    if (time > SHIFT3_START) return time - SHIFT3_START;
    if (time > SHIFT4_START) return time - SHIFT4_START;
    if (time > ENDGAME_START) return time - ENDGAME_START;

    return time;
  }

  /* =========================
  ACTIVE SHIFT
  ========================= */

  /** True when our alliance is the one allowed to score right now. */
  public static boolean isShiftActive() {

    if (!DriverStation.isEnabled()) return false;

    if (DriverStation.isAutonomousEnabled()) return true;

    GameState state = getCurrentGameState();

    // AUTO and TRANSITION: both alliances active
    if (state == GameState.AUTO || state == GameState.TRANSITION) return true;

    // ENDGAME: both alliances active
    if (state == GameState.ENDGAME) return true;

    boolean blueStarts = doesBlueStartShift1();
    boolean weAreBlue = !Robot.isRedAlliance();

    boolean blueActive;

    // Ownership alternates between alliances on every shift
    switch (state) {
      case SHIFT1:
        blueActive = blueStarts;
        break;

      case SHIFT2:
        blueActive = !blueStarts;
        break;

      case SHIFT3:
        blueActive = blueStarts;
        break;

      case SHIFT4:
        blueActive = !blueStarts;
        break;

      default:
        return false;
    }

    return weAreBlue ? blueActive : !blueActive;
  }

  /* =========================
  DASHBOARD UPDATE
  ========================= */

  /** Publishes the shift information for the drivers. Called every loop from Robot. */
  public static void updateDashboard() {

    SmartDashboard.putNumber("MatchTime", getMatchTime());

    SmartDashboard.putNumber("RemainingShiftTime", getRemainingShiftTime());

    SmartDashboard.putBoolean("ShiftActive", isShiftActive());

    SmartDashboard.putString("GameState", getCurrentGameState().name());

    SmartDashboard.putString("ActiveFirst", getActiveFirst());
  }
}
