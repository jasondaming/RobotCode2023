package org.littletonrobotics.frc2023.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakeMotor;

  public IntakeIOSparkMax() {
    intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePercent = intakeMotor.get();
    inputs.intakeCurrent = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
  }
}