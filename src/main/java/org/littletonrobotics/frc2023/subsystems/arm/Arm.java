// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import org.littletonrobotics.frc2023.util.arm.ArmConfig;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmSolverIO solverIo;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmSolverIOInputsAutoLogged solverInputs = new ArmSolverIOInputsAutoLogged();

  private static final String configFilename = "arm_config.json";
  private final String configJson;
  private final ArmConfig config;

  public Arm(ArmIO io, ArmSolverIO solverIo) {
    this.io = io;
    this.solverIo = solverIo;
    io.setBrakeMode(true, true, true);

    // Get config from JSON
    File configFile = new File(Filesystem.getDeployDirectory(), configFilename);
    try {
      configJson = Files.readString(configFile.toPath());
    } catch (IOException e) {
      throw new RuntimeException("Failed to read raw arm config JSON");
    }
    config = ArmConfig.fromJson(configFile);
    io.setConfig(config);
    solverIo.setConfig(configJson);
  }

  public void periodic() {
    io.updateInputs(inputs);
    solverIo.updateInputs(solverInputs);
    Logger.getInstance().processInputs("Arm", inputs);
    Logger.getInstance().processInputs("Arm/Solver", solverInputs);
  }
}
