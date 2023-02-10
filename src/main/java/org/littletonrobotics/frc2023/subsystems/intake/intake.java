// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  private enum gamepiece {
    cone,
    cube,
    nothing
  }

  private gamepiece piece = gamepiece.cone;

  private CANSparkMax intakeMotor;
  
  /** Creates a new intake. */
  public intake() {
    intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCone() {
    piece = gamepiece.cone;
  }

  public void setCube() {
    piece = gamepiece.cube;
  }

  public void intakeCube() {

  }
  
  public void setOutput(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
    Logger.getInstance().recordOutput("Intake/Output", percent);
    Logger.getInstance().recordOutput("Intake/CurrentLimit", amps);
  }
}
