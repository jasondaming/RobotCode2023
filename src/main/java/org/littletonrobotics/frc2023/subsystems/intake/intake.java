// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private enum gamepiece {
    cone(1),
    cube(-1),
    nothing(0);

    private int numVal;

    gamepiece(int numVal) {
      this.numVal = numVal;
    }

    public int getNumVal() {
      return numVal;
    }
  }

  private gamepiece piece = gamepiece.cone;

  /** How many amps the intake can use while picking up */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /** How many amps the intake can use while holding */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /** Percent output for intaking */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /** Percent output for holding */
  static final double INTAKE_HOLD_POWER = 0.07;

  /** Creates a new intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }

  public void setCone() {
    piece = gamepiece.cone;
  }

  public void setCube() {
    piece = gamepiece.cube;
  }

  public void intakeCube() {
    io.setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void intakeCone() {
    io.setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void outtakeCube() {
    io.setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void outtakeCone() {
    io.setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void holdCube() {
    io.setIntakeMotor(INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void holdCone() {
    io.setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void pickup() {
    io.setIntakeMotor(INTAKE_OUTPUT_POWER * piece.getNumVal(), INTAKE_CURRENT_LIMIT_A);
  }

  public void place() {
    io.setIntakeMotor(-INTAKE_OUTPUT_POWER * piece.getNumVal(), INTAKE_CURRENT_LIMIT_A);
  }
}
