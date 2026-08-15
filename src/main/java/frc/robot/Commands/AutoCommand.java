package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

/**
 * Base class for every autonomous routine. Beyond running as a command group, it exposes the path
 * poses so the dashboard can preview the route before the match starts.
 */
public abstract class AutoCommand extends SequentialCommandGroup {

  /** Every pose along the routine, used to draw the preview on the Field2d widget. */
  public abstract List<Pose2d> getAllPathPoses();

  /** Pose the robot must be placed at before the routine starts. */
  public abstract Pose2d getStartingPose();
}
