# FRC 2026 Robot Code - Project Overview

This is the software repository for a FIRST Robotics Competition (FRC) 2026 robot. It is a Java-based project utilizing the WPILib framework along with several vendor libraries to control the robot's hardware.

## Project Structure & Technologies

- **Language:** Java 17
- **Build System:** Gradle (via GradleRIO 2026.2.1)
- **Framework:** WPILib (Command-Based architecture)
- **Key Vendor Libraries:**
  - **CTRE Phoenix 6** (Motor controllers/Sensors)
  - **REVLib** (Motor controllers/Sensors)
  - **AdvantageKit** (Data logging and replay framework)
  - **PathPlanner** (Autonomous path generation and following)
  - **URCL** (Unofficial REV Compatible Logger)

## Subsystems

Based on the `RobotContainer`, the robot is composed of the following main subsystems:
- **Swerve:** The drivetrain, likely using CTRE Swerve hardware/API (TunerConstants).
- **Intake:** Handles acquiring game pieces. Includes pivoting functionality.
- **Shooter:** Handles launching game pieces.
- **Hooper:** A hopper/indexer mechanism to feed the shooter.
- **Vision:** Handles AprilTag/target tracking, likely using PhotonVision or similar via AdvantageKit (`LocalADStarAK`).
- **Climber:** (Currently commented out in code) For endgame climbing.

## Building and Running

The project uses the standard GradleRIO wrapper for building and deploying code to the RoboRIO.

### Essential Commands

Run these commands from the terminal in the root directory:

- **Build the project:**
  ```bash
  ./gradlew build
  ```
- **Deploy code to the RoboRIO:**
  ```bash
  ./gradlew deploy
  ```
- **Run the simulation (Desktop):**
  ```bash
  ./gradlew simulateJava
  ```
- **Run tests:**
  ```bash
  ./gradlew test
  ```
- **Format code (Spotless):**
  ```bash
  ./gradlew spotlessApply
  ```

## Development Conventions

- **Code Formatting:** The project enforces formatting using the `Spotless` Gradle plugin with `googleJavaFormat()`. Always run `./gradlew spotlessApply` before committing changes to avoid CI failures.
- **Logging:** The project is configured with **AdvantageKit** for comprehensive telemetry and replay support. Data is logged from inputs, state, and outputs.
- **Simulation:** The codebase includes robust simulation support, particularly for `IntakeIOSim` and `ShooterSim`, allowing testing without hardware.
- **Autonomous:** Autonomous routines are built using `PathPlanner` and are stored in the `src/main/deploy/pathplanner` directory. Commands are registered via `NamedCommands` in `RobotContainer.java`.
