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
 * Mirror of LCAutoLeft, starting from the right side of the field. The three paths are loaded only
 * to draw the dashboard preview.
 */
public class LCAutoRight extends AutoCommand {

  private final PathPlannerPath LCIntake1DS1Right;
  private final PathPlannerPath LCIntake2DS2Right;
  private final PathPlannerPath LCIntake3DS3Right;

  public LCAutoRight() {
    LCIntake1DS1Right = loadPath("LCIntake1DS1Right");
    LCIntake2DS2Right = loadPath("LCIntake2DS2Right");
    LCIntake3DS3Right = loadPath("LCIntake3DS3Right");

    // "LCAutoRight" must be the exact name of the .auto file in PathPlanner
    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("LCAutoRight"))));
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
            safeGetPathPoses(LCIntake1DS1Right),
            safeGetPathPoses(LCIntake2DS2Right),
            safeGetPathPoses(LCIntake3DS3Right))
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  /** Returns an empty list when a path failed to load, so the preview still works. */
  private List<Pose2d> safeGetPathPoses(PathPlannerPath path) {
    return path != null ? path.getPathPoses() : new ArrayList<>();
  }

  /** Starting pose of the routine. Note it uses the second path, not the first. */
  @Override
  public Pose2d getStartingPose() {
    if (LCIntake2DS2Right != null) {
      return LCIntake2DS2Right.getStartingDifferentialPose();
    }
    DriverStation.reportError("First path is null. Returning default starting pose.", true);
    return new Pose2d();
  }
}
