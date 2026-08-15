package frc.robot.Commands.Paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommand;
import java.util.Collections;
import java.util.List;

/** Empty autonomous option: the robot does nothing and nothing is drawn on the preview. */
public class NonePath extends AutoCommand {
  public NonePath() {
    addCommands(Commands.none());
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Collections.emptyList();
  }

  @Override
  public Pose2d getStartingPose() {
    return new Pose2d();
  }
}
