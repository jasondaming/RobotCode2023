// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI extends HandheldOI {
  private final CommandXboxController controller;

  public SingleHandheldOI(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getLeftDriveX() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double getLeftDriveY() {
    return -controller.getRawAxis(0);
  }

  @Override
  public double getRightDriveX() {
    return -controller.getRawAxis(3);
  }

  @Override
  public double getRightDriveY() {
    return -controller.getRawAxis(2);
  }

  @Override
  public void setDriverRumble(double percent) {
    controller.getHID().setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    controller.getHID().setRumble(RumbleType.kRightRumble, percent);
  }
}
