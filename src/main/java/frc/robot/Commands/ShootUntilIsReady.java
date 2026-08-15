package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.ShotCalculator;
import frc.robot.Utils.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Hooper.Hooper;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * Spins the shooter from the distance tables and only feeds balls once both the flywheels are at
 * speed and the drivetrain is aimed. Prevents shots that would miss because the robot moved.
 */
public class ShootUntilIsReady extends Command {
  private final Shooter m_shooter;
  private final Hooper m_hooper;

  private final ShotCalculator m_shotCalc = ShotCalculator.getInstance();

  public ShootUntilIsReady(Shooter shooter, Hooper hooper) {
    this.m_shooter = shooter;
    this.m_hooper = hooper;

    addRequirements(m_shooter, m_hooper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.shootAndMoveLoLVoid();

    ShootingParameters params = m_shotCalc.getParameters();

    boolean driveIsReady = false;
    boolean shooterIsReady = m_shooter.atSpeed(20);

    // Heading check only makes sense when the target is within the valid distance range.
    if (params.isValid()) {
      Rotation2d targetAngle = params.turretAngle();
      driveIsReady = frc.robot.Utils.RobotState.getInstance().atOrbitAngle(3, targetAngle);
    }

    // Feed only when both conditions hold, otherwise hold the balls back.
    if (driveIsReady && shooterIsReady) {
      m_hooper.setVoltageVoid(12, 10);
    } else {
      m_hooper.stopvoid();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopVoid();
    m_hooper.stopvoid();
  }
}
