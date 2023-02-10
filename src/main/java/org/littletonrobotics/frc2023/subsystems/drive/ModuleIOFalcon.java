// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2023.Constants;

public class ModuleIOFalcon implements ModuleIO {
  private final WPI_TalonFX driveFalcon;
  private final WPI_TalonFX turnFalcon;

  private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfiguration = new TalonFXConfiguration();

  private final double driveSensorPositionCoefficient =
      SdsModuleConfigurations.MK4_L1.getDriveReduction() / 2048.0;
  private final double driveSensorVelocityCoefficient =
      driveSensorPositionCoefficient * 10.0 * 60.0;
  private final double turnSensorPositionCoefficient =
      2.0 * Math.PI * SdsModuleConfigurations.MK4_L1.getSteerReduction() / 2048.0;
  private final double turnSensorVelocityCoefficient = turnSensorPositionCoefficient * 10.0 * 60.0;

  private final CANCoder turnAbsoluteEncoder;
  private final CANCoderConfiguration config = new CANCoderConfiguration();

  private static double angle = 0;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;

  public ModuleIOFalcon(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2023P:
        switch (index) {
          case 0:
            driveFalcon = new WPI_TalonFX(11);
            turnFalcon = new WPI_TalonFX(12);
            turnAbsoluteEncoder = new CANCoder(13);
            absoluteEncoderOffset = -28.531;
            break;
          case 1:
            driveFalcon = new WPI_TalonFX(21);
            turnFalcon = new WPI_TalonFX(22);
            turnAbsoluteEncoder = new CANCoder(23);
            absoluteEncoderOffset = -174.638;
            break;
          case 2:
            driveFalcon = new WPI_TalonFX(31);
            turnFalcon = new WPI_TalonFX(32);
            turnAbsoluteEncoder = new CANCoder(33);
            absoluteEncoderOffset = -99.492;
            break;
          case 3:
            driveFalcon = new WPI_TalonFX(41);
            turnFalcon = new WPI_TalonFX(42);
            turnAbsoluteEncoder = new CANCoder(43);
            absoluteEncoderOffset = -238;
            break;
          default:
            throw new RuntimeException("Invalid module index for ModuleIOFalcon");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOFalcon");
    }
    driveFalcon.configFactoryDefault();
    turnFalcon.configFactoryDefault();

    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = Math.toDegrees(absoluteEncoderOffset);
    config.sensorDirection = false;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    turnAbsoluteEncoder.configAllSettings(config, 250);

    driveConfiguration.supplyCurrLimit.enable = true;
    driveConfiguration.supplyCurrLimit.currentLimit = 80.0;
    turnConfiguration.supplyCurrLimit.enable = true;
    turnConfiguration.supplyCurrLimit.currentLimit = 20.0;
    driveConfiguration.voltageCompSaturation = 12.0;
    turnConfiguration.voltageCompSaturation = 12.0;
    turnConfiguration.slot0.kP = 0.02;
    turnConfiguration.slot0.kI = 0.0;
    turnConfiguration.slot0.kD = 0.01;
    driveFalcon.configAllSettings(driveConfiguration);
    turnFalcon.configAllSettings(turnConfiguration);

    driveFalcon.setInverted(false);
    turnFalcon.setInverted(isTurnMotorInverted);

    driveFalcon.setSensorPhase(true);
    turnFalcon.setSensorPhase(true);

    // driveFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, 250);
    // turnFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, 250);

    turnFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250);
    turnFalcon.setSelectedSensorPosition(
        getAbsoluteAngleRetry() / turnSensorPositionCoefficient, 0, 250);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(
            driveFalcon.getSelectedSensorPosition() * driveSensorPositionCoefficient);
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            driveFalcon.getSelectedSensorVelocity() * driveSensorVelocityCoefficient);
    inputs.driveAppliedVolts = driveFalcon.getMotorOutputVoltage();
    inputs.driveCurrentAmps = new double[] {driveFalcon.getStatorCurrent()};
    inputs.driveTempCelcius = new double[] {driveFalcon.getTemperature()};

    inputs.turnAbsolutePositionRad =
        Math.toRadians(turnAbsoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset);
    inputs.turnPositionRad =
        Units.rotationsToRadians(
            turnFalcon.getSelectedSensorPosition() * turnSensorPositionCoefficient);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            turnFalcon.getSelectedSensorVelocity() * turnSensorVelocityCoefficient);
    inputs.turnAppliedVolts = turnFalcon.getMotorOutputVoltage();
    inputs.turnCurrentAmps = new double[] {turnFalcon.getStatorCurrent()};
    inputs.turnTempCelcius = new double[] {turnFalcon.getTemperature()};
  }

  public void setDriveVoltage(double volts) {
    driveFalcon.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnFalcon.setVoltage(volts);
  }

  public void setTurnAngle(double angle) {
    turnFalcon.set(ControlMode.Position, angle / turnSensorPositionCoefficient);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveFalcon.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnFalcon.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public double getAbsoluteAngleRetry() {
    double time = Timer.getFPGATimestamp();
    boolean success = false;
    boolean timeout = false;
    do {
      angle = getAbsoluteAngle();
      success = turnAbsoluteEncoder.getLastError() == ErrorCode.OK;
      timeout = Timer.getFPGATimestamp() - time > 8;
    } while (!success && !timeout);

    return angle;
  }

  public double getAbsoluteAngle() {
    angle = Math.toRadians(turnAbsoluteEncoder.getPosition());

    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }
}
