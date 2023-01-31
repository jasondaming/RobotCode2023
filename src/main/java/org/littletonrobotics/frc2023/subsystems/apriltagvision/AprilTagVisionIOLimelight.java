// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.apriltagvision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  NetworkTable limelightTable;

  public AprilTagVisionIOLimelight(String identifier) {
    limelightTable = NetworkTableInstance.getDefault().getTable(identifier);
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.timestamps = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    inputs.frames = new double[0][];
    inputs.fps = limelightTable.getEntry("botpose").getInteger(0);
  }
}
