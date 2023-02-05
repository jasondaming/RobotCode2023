// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import org.littletonrobotics.frc2023.oi.HandheldOI;
import org.littletonrobotics.frc2023.oi.OISelector;
import org.littletonrobotics.frc2023.oi.OverrideOI;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIOPigeon;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOFalcon;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private AprilTagVision aprilTagVision;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI();

  // Choosers
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // Grid Selection Variables
  private int grid = 0;
  private int position = 0;
  private int column = 0;
  private int level = 0;

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          break;
        case ROBOT_2023P:
          drive =
              new Drive(
                  new GyroIOPigeon(),
                  new ModuleIOFalcon(0),
                  new ModuleIOFalcon(1),
                  new ModuleIOFalcon(2),
                  new ModuleIOFalcon(3));
          aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));
          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (aprilTagVision == null) {
      // In replay, match the number of instances for each robot
      switch (Constants.getRobot()) {
        case ROBOT_2023P:
          aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
          break;
        default:
          aprilTagVision = new AprilTagVision();
          break;
      }
    }

    // Set up subsystems
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> handheldOI.getLeftDriveX(),
            () -> handheldOI.getLeftDriveY(),
            () -> handheldOI.getRightDriveY()));
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addVisionData);

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", null);
    autoChooser.addOption("Reset Odometry", new InstantCommand(() -> drive.setPose(new Pose2d())));
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drive,
            true,
            new FeedForwardCharacterizationData("drive"),
            drive::runCharacterizationVolts,
            drive::getCharacterizationVelocity));

    autoChooser.addOption(
        "Test Trajectory",
        new InstantCommand(() -> drive.setPose(new Pose2d()))
            .andThen(
                new DriveTrajectory(
                    drive,
                    List.of(
                        Waypoint.fromHolonomicPose(new Pose2d()),
                        Waypoint.fromHolonomicPose(
                            new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(45.0)))))));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    // *** DRIVER CONTROLS ***
    /*     DriveTrajectory traj;
    if (DriverStation.getAlliance() == Alliance.Red) {
      var poses = new ArrayList<>(FieldConstants.redACorridor);
      poses.add(FieldConstants.scoringRed.get(column));
      traj = poseTrajectory(poses);
    } else {
      var poses = new ArrayList<>(FieldConstants.blueACorridor);
      poses.add(FieldConstants.scoringBlue.get(column));
      traj = poseTrajectory(poses);
    }
    List<Pose3d> poses = FieldConstants.corridor.get(FieldConstants.Corridor.A).get(Alliance.Red);
    traj.andThen(null)
    handheldOI.getScoreA().whileTrue(scoringTraj()); */

    // *** OPERATOR CONTROLS ***
    overrideOI.setGridLeft().onTrue(Commands.runOnce(() -> grid = 0).andThen(() -> updateGrid()));
    overrideOI.setGridCenter().onTrue(Commands.runOnce(() -> grid = 3).andThen(() -> updateGrid()));
    overrideOI.setGridRight().onTrue(Commands.runOnce(() -> grid = 6).andThen(() -> updateGrid()));
  }

  public void updateGrid() {
    if (position < 4) {
      level = 3;
    } else if (position > 3 && position < 7) {
      level = 2;
    } else {
      level = 1;
    }
    var positionadd = 0;
    if (position % 3 == 0) {
      positionadd = 3;
    } else {
      positionadd = position % 3;
    }
    column = grid + positionadd;
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private DriveTrajectory poseTrajectory(List<Pose3d> poses) {
    List<Waypoint> waypoints = new ArrayList<>();
    for (Pose3d pose : poses) {
      waypoints.add(Waypoint.fromHolonomicPose(pose.toPose2d()));
    }
    return new DriveTrajectory(drive, waypoints);
  }
}
