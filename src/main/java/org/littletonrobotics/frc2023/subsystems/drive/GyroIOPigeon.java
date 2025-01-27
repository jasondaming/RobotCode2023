// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon implements GyroIO {
  private final PigeonIMU pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon() {
    switch (Constants.getRobot()) {
      case ROBOT_2023P:
        TalonSRX pigeonTalon = new TalonSRX(5);
        pigeon = new PigeonIMU(pigeonTalon);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIOPigeon");
    }

    pigeon.configFactoryDefault();
    pigeon.setYaw(0.0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  }
}
