package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.ShotCalculator;
import frc.robot.Utils.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Keeps the robot pointed at the shooting target while the driver still controls translation. The
 * heading comes from ShotCalculator, so it accounts for the robot's own velocity.
 */
public class OrbitinWithMovement extends Command {
  private final CommandSwerveDrivetrain m_drive;

  /** Driver translation inputs, normally scaled down by the caller. */
  private final Supplier<Double> m_xSpd, m_ySpd;

  private final PIDController m_rotationPID = new PIDController(5, 0, 0);
  private final SwerveRequest.FieldCentric m_request = new SwerveRequest.FieldCentric();
  private final ShotCalculator m_shotCalc = ShotCalculator.getInstance();

  public OrbitinWithMovement(
      CommandSwerveDrivetrain drive, Supplier<Double> xSpd, Supplier<Double> ySpd) {

    this.m_drive = drive;
    this.m_xSpd = xSpd;
    this.m_ySpd = ySpd;

    // Wrap around so the shortest way to the target heading is always taken.
    m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    // 1 degree of error tolerance, converted to radians
    m_rotationPID.setTolerance(Math.toRadians(1.0));

    addRequirements(drive);
  }

  /** True when the robot is pointing at the target within the tolerance. */
  public boolean atTarget() {
    return m_rotationPID.atSetpoint();
  }

  @Override
  public void execute() {
    var robotPose = m_drive.getState().Pose;
    ShootingParameters params = m_shotCalc.getParameters();

    double rotationOutput = 0.0;

    if (params.isValid()) {
      // 180 degree offset because the shooter fires out of the back of the robot.
      Rotation2d targetAngle = params.turretAngle().minus(Rotation2d.fromDegrees(180));

      // Feedforward term keeps up with a target that moves as the robot drives.
      rotationOutput =
          m_rotationPID.calculate(robotPose.getRotation().getRadians(), targetAngle.getRadians())
              + params.turretVelocity();
    } else {
      // No valid target, clear the accumulated PID state instead of aiming at garbage.
      m_rotationPID.reset();
    }

    double maxSpeed = 10.0;

    m_drive.setControl(
        m_request
            .withVelocityX(-m_xSpd.get() * maxSpeed)
            .withVelocityY(-m_ySpd.get() * maxSpeed)
            .withRotationalRate(rotationOutput));

    m_drive.xBrakeCommand();

    SmartDashboard.putNumber("Orbiting/Target Angle", rotationOutput);
    SmartDashboard.putNumber("Orbiting/Actual Angle", m_drive.getCurrentAngle().getDegrees());
    SmartDashboard.putBoolean("Orbiting/Is Aimed", atTarget());

    // Invalidate the cache so the next loop recomputes with the new pose.
    m_shotCalc.clearShootingParameters();
  }

  /** Runs until the button is released. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
