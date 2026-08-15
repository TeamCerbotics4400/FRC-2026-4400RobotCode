package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

/**
 * ShooterIO implementation for the real robot. Four flywheel TalonFX motors, where the upper right
 * one is the leader and the other three follow it, plus a Motion Magic controlled hood.
 */
public class ShooterFX implements ShooterIO {

  private final TalonFX upperMotorRight =
      new TalonFX(ShooterConstants.flywheeUPlMotorIDRight, "canivore");
  private final TalonFX downMotorRight =
      new TalonFX(ShooterConstants.flywheeDownMotorIDRight, "canivore");

  private final TalonFX upperMotorLeft =
      new TalonFX(ShooterConstants.flywheeUPlMotorIDLeft, "canivore");
  private final TalonFX downMotorLeft =
      new TalonFX(ShooterConstants.flywheeDownMotorIDLeft, "canivore");

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  final NeutralOut m_requestNeutral = new NeutralOut();

  private final TalonFXConfiguration upperRightConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration downRightConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration upperLeftConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration downLeftConfiguration = new TalonFXConfiguration();

  private double targetRPM = 0.0;

  private final TalonFX hoodMotor =
      new TalonFX(frc.robot.Constants.HoodConstants.hoodMotorID, "canivore");
  private TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();

  private final MotionMagicVoltage m_requestHood = new MotionMagicVoltage(0);

  /** Hood position request using torque current FOC with the slot 1 gains. */
  private final PositionTorqueCurrentFOC m_requestHoodFOC =
      new PositionTorqueCurrentFOC(0).withSlot(1);

  private static double desiredDegrees = 0.0;

  private final StatusSignal<Current> upperMotorRightCurrent = upperMotorRight.getSupplyCurrent();

  public ShooterFX() {
    // Basic motor configuration
    upperRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    upperRightConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
    upperRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    downRightConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
    downRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    upperLeftConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
    upperLeftConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    downLeftConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
    downLeftConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Slot 1: alternative PID (kept for potential use)
    upperRightConfiguration.Slot1.kS = 0.3;
    upperRightConfiguration.Slot1.kV = 0.5;
    upperRightConfiguration.Slot1.kA = 0.065;
    upperRightConfiguration.Slot1.kP = 55.0; // 6.0394;//4.5178;
    upperRightConfiguration.Slot1.kI = 0;
    upperRightConfiguration.Slot1.kD = 0;

    var motionMagicConfigs = upperRightConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        1500; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_request.EnableFOC = true;

    m_request.UpdateFreqHz = 1000;

    upperRightConfiguration
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(100))
        .withPeakReverseTorqueCurrent(Amps.of(-100));
    upperRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    upperMotorRight.getConfigurator().apply(upperRightConfiguration);
    downMotorRight.getConfigurator().apply(downRightConfiguration);
    upperMotorLeft.getConfigurator().apply(upperRightConfiguration);
    downMotorLeft.getConfigurator().apply(downRightConfiguration);

    // Only the upper right motor is commanded, the other three mirror it.
    // Aligned spins the same way, Opposed spins the other way (motors face each other).
    downMotorRight.setControl(
        new Follower(upperMotorRight.getDeviceID(), MotorAlignmentValue.Aligned));

    upperMotorLeft.setControl(
        new Follower(upperMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));

    downMotorLeft.setControl(
        new Follower(upperMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));

    // Publish initial tuning values & telemetry
    SmartDashboard.putNumber("Shooter/kP", upperRightConfiguration.Slot1.kP);
    SmartDashboard.putNumber("Shooter/kI", upperRightConfiguration.Slot1.kI);
    SmartDashboard.putNumber("Shooter/kD", upperRightConfiguration.Slot1.kD);
    SmartDashboard.putNumber("Shooter/kS", upperRightConfiguration.Slot1.kS);
    SmartDashboard.putNumber("Shooter/kV", upperRightConfiguration.Slot1.kV);
    SmartDashboard.putNumber("Shooter/Target RPM", 0.0);

    SmartDashboard.putNumber(
        "Shooter/VoltageUpper", upperMotorRight.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/VoltageLower", downMotorLeft.getMotorVoltage().getValueAsDouble());

    hoodConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Assumes the hood starts at its 20 degree mechanical minimum at boot.
    hoodMotor.setPosition(0);

    hoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    MotionMagicConfigs mmConfigs = hoodConfiguration.MotionMagic;
    mmConfigs
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
        .withMotionMagicAcceleration(10)
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    // Torq current FOC
    hoodConfiguration.Slot1.kS = 0; // 30
    hoodConfiguration.Slot1.kV = 0;
    hoodConfiguration.Slot1.kA = 0;
    hoodConfiguration.Slot1.kP = 100; // 80
    hoodConfiguration.Slot1.kI = 0;
    hoodConfiguration.Slot1.kD = 4; // 6

    // Software stops so the hood cannot drive past its physical travel.
    hoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.9;
    hoodConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    hoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodMotor.getConfigurator().apply(hoodConfiguration);

    SmartDashboard.putNumber("Hood/Desired Degrees", desiredDegrees + 20);

    /* Status signal for a better update of frequency */
    BaseStatusSignal.setUpdateFrequencyForAll(1000, upperMotorRightCurrent);

    upperRightConfiguration.Feedback.VelocityFilterTimeConstant = 0.005;
  }

  /* Shooter */

  @Override
  public void runShooter() {
    double targetRPS = targetRPM / 60.0;
    // Use Phoenix native velocity control (slot 1 is chosen here)
    upperMotorRight.setControl(new VelocityTorqueCurrentFOC(targetRPS).withSlot(1));
    // downMotor follows upperMotor
  }

  /** Velocity control from a supplier. Also stores the value so it can be logged. */
  @Override
  public void runShotLOL(Supplier<Double> targetRPMs) {
    double targetRPS = targetRPMs.get() / 60.0;
    targetRPM = targetRPMs.get();
    // Use Phoenix native velocity control (slot 1 is chosen here)
    upperMotorRight.setControl(new VelocityTorqueCurrentFOC(targetRPS).withSlot(1));
    // downMotor follows upperMotor
  }

  /* Hood */

  /** Converts a hood angle to motor rotations. 20 degrees is the mechanical zero. */
  @Override
  public void goToAngle(Supplier<Double> angleDegrees) {
    double motorRotations = (angleDegrees.get() - 20) / HoodConstants.kHoodDegreesPerMotorRotation;
    m_requestHoodFOC.Position = motorRotations;
    hoodMotor.setControl(m_requestHoodFOC);
  }

  @Override
  public void runVolts(double volts) {
    upperMotorRight.setControl(new VoltageOut(volts));
  }

  @Override
  public void stopMotor() {
    targetRPM = 0;
    upperMotorRight.stopMotor();
  }

  @Override
  public double getStator() {
    return upperMotorRight.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {

    inputs.hoodAngleDegrees =
        (hoodMotor.getPosition().getValueAsDouble() * HoodConstants.kHoodDegreesPerMotorRotation)
            + 20;

    // Phoenix reports rotations per second, so multiply by 60 to get RPM.
    inputs.shooterRPMs =
        ((upperMotorRight.getVelocity().getValueAsDouble() * 60)
                + (downMotorRight.getVelocity().getValueAsDouble() * 60)
                + (upperMotorLeft.getVelocity().getValueAsDouble() * 60)
                + (downMotorLeft.getVelocity().getValueAsDouble() * 60))
            / 4;

    inputs.upRightFWRPM = upperMotorRight.getVelocity().getValueAsDouble() * 60;
    inputs.downRightFWRPM = downMotorRight.getVelocity().getValueAsDouble() * 60;
    inputs.upLeftFWRPM = upperMotorLeft.getVelocity().getValueAsDouble() * 60;
    inputs.downLeftFWRPM = upperMotorLeft.getVelocity().getValueAsDouble() * 60;

    inputs.ShooterVoltageUpperRight = upperMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.ShooterVoltageDownRight = downMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentUpperRight = getStator();
    inputs.statorCurrenDownRight = downMotorRight.getStatorCurrent().getValueAsDouble();
    inputs.ShooterVoltageUpperLeft = upperMotorLeft.getMotorVoltage().getValueAsDouble();
    inputs.ShooterVoltageDownLeft = downMotorLeft.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentUpperLeft = downMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.statorCurrenDownLeft = downMotorLeft.getStatorCurrent().getValueAsDouble();

    inputs.targetRPM = targetRPM;

    // Report current slot-0 gains
    inputs.kp = upperRightConfiguration.Slot0.kP;
    inputs.kd = upperRightConfiguration.Slot0.kD;
    inputs.kv = upperRightConfiguration.Slot0.kV;
    inputs.ks = upperRightConfiguration.Slot0.kS;

    SmartDashboard.putNumber(
        "Shooter/Current RPM", upperMotorRight.getVelocity().getValueAsDouble() * 60);
    double desiredDeg = SmartDashboard.getNumber("Hood/Desired Degrees", 0.0) - 20;
    SmartDashboard.putNumber(
        "Actual Degrees",
        hoodMotor.getPosition().getValueAsDouble() * HoodConstants.kHoodDegreesPerMotorRotation
            + 20);

    // Report current slot-0 gains
    inputs.hkp = hoodConfiguration.Slot1.kP;
    inputs.hkd = hoodConfiguration.Slot1.kD;
    inputs.hKv = hoodConfiguration.Slot1.kV;
    inputs.hks = hoodConfiguration.Slot1.kS;

    SmartDashboard.putNumber("Actual rotationsMotor", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Desired Rotations Theory", m_requestHood.Position);
    if (desiredDegrees != desiredDeg) {
      desiredDegrees = desiredDeg;
    }
  }
}
