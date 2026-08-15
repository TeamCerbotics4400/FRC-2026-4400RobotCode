package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * Default teleop drive command. Takes the joystick axes, smooths them with slew rate limiters and
 * sends field centric velocities to the swerve drivetrain.
 */
public class DriveCommand extends Command {

  frc.robot.subsystems.Swerve.CommandSwerveDrivetrain m_drive;

  /** Joystick axes, already negated by the caller to match the field frame. */
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;

  /** Limit how fast the commanded speed can change, so the robot does not tip on sudden inputs. */
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private double MaxSpeed =
      frc.robot.subsystems.Swerve.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  /** Field centric request with a 10% deadband so stick drift does not move the robot. */
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DriveCommand(
      frc.robot.subsystems.Swerve.CommandSwerveDrivetrain m_drive,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction) {

    this.m_drive = m_drive;

    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    this.xLimiter = new SlewRateLimiter(MaxSpeed);
    this.yLimiter = new SlewRateLimiter(MaxSpeed);
    this.turningLimiter = new SlewRateLimiter(MaxAngularRate);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {}

  /** Scales the limited joystick values to real velocities and applies them. */
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    xSpeed = xLimiter.calculate(xSpeed) * MaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * MaxSpeed;

    turningSpeed = turningLimiter.calculate(turningSpeed) * MaxAngularRate;

    m_drive.setControl(
        fieldCentric.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(turningSpeed));
  }

  @Override
  public void end(boolean interrupted) {}

  /** Never finishes: it is the default command and only stops when another command takes over. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
