// SecondPickAuto

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
 * Autonomous routine that picks up a second load from the depot before shooting. The three paths
 * are loaded only to draw the dashboard preview.
 */
public class SecondPickAuto extends AutoCommand {
  private final PathPlannerPath Intake1SP1;
  private final PathPlannerPath IntakeDepotSP2;
  private final PathPlannerPath ShootDepotSP3;

  public SecondPickAuto() {
    Intake1SP1 = loadPath("Intake1SP1");
    IntakeDepotSP2 = loadPath("IntakeDepotSP2");
    ShootDepotSP3 = loadPath("ShootDepotSP3");

    // "SecondPickAuto" must be the exact name of the .auto file in PathPlanner
    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("SecondPickAuto"))));
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
            safeGetPathPoses(Intake1SP1),
            safeGetPathPoses(IntakeDepotSP2),
            safeGetPathPoses(ShootDepotSP3))
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
    if (Intake1SP1 != null) {
      return Intake1SP1.getStartingDifferentialPose();
    }
    DriverStation.reportError("First path is null. Returning default starting pose.", true);
    return new Pose2d();
  }
}
