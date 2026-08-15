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
 * Default autonomous routine: one scoring cycle plus a trip to the depot. The two paths are loaded
 * only to draw the dashboard preview.
 */
public class LagunaAuto extends AutoCommand {

  private final PathPlannerPath intake1NS1;
  private final PathPlannerPath shoot1NS2;

  public LagunaAuto() {
    intake1NS1 = loadPath("Intake1NS1");
    shoot1NS2 = loadPath("Intake&Shoot1NS1");

    // "LagunaAuto" must be the exact name of the .auto file
    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("LagunaAuto"))));
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
    return Stream.of(safeGetPathPoses(intake1NS1), safeGetPathPoses(shoot1NS2))
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
    if (intake1NS1 != null) {
      return intake1NS1.getStartingDifferentialPose();
    }
    DriverStation.reportError("First path is null. Returning default starting pose.", true);
    return new Pose2d();
  }
}
