// DoubleSwipeAuto

package frc.robot.Commands.Paths;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommand;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Autonomous routine with three intake and shoot cycles. The three paths are loaded only to draw
 * the dashboard preview, the routine itself is run by PathPlanner.
 */
public class DoubleSwipeAuto extends AutoCommand {
  private final PathPlannerPath Intake1DS1;
  private final PathPlannerPath Intake2DS2;
  private final PathPlannerPath Intake3DS3;

  public DoubleSwipeAuto() {
    Intake1DS1 = loadPath("Intake1DS1");
    Intake2DS2 = loadPath("Intake2DS2");
    Intake3DS3 = loadPath("Intake3DS3");

    // "DoubleSwipeAuto" must be the exact name of the .auto file in PathPlanner
    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("DoubleSwipeAuto"))));
  }

  /** Loads a path file, reporting to the Driver Station instead of crashing if it is missing. */
  private PathPlannerPath loadPath(String fileName) {
    try {
      return PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path: " + fileName + " - " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  /** All poses of the routine, concatenated in the order they are driven. */
  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            safeGetPathPoses(Intake1DS1),
            safeGetPathPoses(Intake2DS2),
            safeGetPathPoses(Intake3DS3))
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  /** Returns an empty list when a path failed to load, so the preview still works. */
  private List<Pose2d> safeGetPathPoses(PathPlannerPath path) {
    return path != null ? path.getPathPoses() : new ArrayList<>();
  }

  /** Starting pose of the routine, taken from the first path. */
  @Override
  public Pose2d getStartingPose() {
    if (Intake1DS1 != null) {
      return Intake1DS1.getStartingDifferentialPose();
    }
    DriverStation.reportError("First path is null. Returning default starting pose.", true);
    return new Pose2d();
  }
}
