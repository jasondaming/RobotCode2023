// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI extends HandheldOI {
  private final CommandXboxController controller;

  public SingleHandheldOI(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getLeftDriveX() {
    return -controller.getLeftY();
  }

  @Override
  public double getLeftDriveY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRightDriveX() {
    return -controller.getRightY();
  }

  @Override
  public double getRightDriveY() {
    return -controller.getRightX();
  }

  @Override
  public Trigger resetGyro() {
    return controller.start();
  }

  @Override
  public Trigger getScoreA() {
    return controller.x();
  }

  @Override
  public Trigger getScoreB() {
    return controller.a();
  }

  @Override
  public Trigger getLoadA() {
    return controller.y();
  }

  @Override
  public Trigger getLoadB() {
    return controller.b();
  }

  @Override
  public void setDriverRumble(double percent) {
    controller.getHID().setRumble(RumbleType.kRightRumble, percent);
  }
}
