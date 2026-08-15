// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Program entry point. Do not put robot code here, use Robot instead. */
public final class Main {
  private Main() {}

  /** Starts the WPILib framework using Robot as the robot implementation. */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
