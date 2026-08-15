package frc.robot.subsystems.Intake;

// Copyleft (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Draws the intake pivot in AdvantageScope as a 2D mechanism and a 3D pose. */
public class IntakeVisualizer {

  /** Spool radius used to turn encoder rotations into linear travel. Tune to the real mechanism. */
  private static final double SPOOL_RADIUS_METERS = 0.02;

  // private final LoggedMechanism2d root;
  private final LoggedMechanismLigament2d intakeLigament;

  @AutoLogOutput private final LoggedMechanism2d intakeMechanism;

  public IntakeVisualizer(String name, Color color) {

    intakeMechanism = new LoggedMechanism2d(1.5, 0.75, new Color8Bit(Color.kBlack));

    LoggedMechanismRoot2d root = intakeMechanism.getRoot("intakeRoot", 0.75, 0.4);

    intakeLigament =
        new LoggedMechanismLigament2d(
            "IntakeVisual", Inches.of(19.1), Degrees.of(-21), 3, new Color8Bit(color));

    // root = intakeMechanism.getRoot("intakePivot", 0.75, 0.4);

    /*intakeLigament =
    root.append(
        new LoggedMechanismLigament2d(
            "IntakeVisual",
            Inches.of(19.1), // base length of the arm
            Degrees.of(-21),
            3,
            new Color8Bit(color)));*/

    root.append(intakeLigament);
  }

  /**
   * Updates the drawing from the measured pivot position.
   *
   * @param motorRotations raw encoder position (rotations)
   */
  public void setIntakePosition(double motorRotations) {

    // rotations to physical meters
    double posMeters = motorRotations * 2.0 * Math.PI * SPOOL_RADIUS_METERS;

    Distance pos = Meters.of(posMeters);

    // update the visual length
    intakeLigament.setLength(pos.plus(Inches.of(19.1)));

    // 3D pose for the robot model, clamped so the drawing does not overextend.
    Logger.recordOutput(
        "Intake/PivotPose", new Pose3d(Math.min(motorRotations, 0.3), 0, 0, Rotation3d.kZero));

    // SmartDashboard.putNumber("Intake stest", motorRotations);
  }
}
