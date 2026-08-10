// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawkLib.hardware.motor.MotorIO;
import frc.hawkLib.hardware.motor.MotorIO.MotorInputs;
import frc.o2026.Configs;
import frc.o2026.Constants;

public class SwerveModule extends SubsystemBase {
  private final MotorIO m_drivingMotor;
  private final MotorIO m_angleMotor;
  private final CANcoder m_angleAbsoluteEncoder;

  private MotorInputs m_drivingMotorInputs;
  private MotorInputs m_angleMotorInputs;

  public SwerveModule(
      MotorIO driveMotor, MotorIO angleMotor, int angleEncoderCanId, Angle angleOffset) {

    m_drivingMotor = driveMotor;
    m_angleMotor = angleMotor;
    m_angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

    m_angleAbsoluteEncoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(angleOffset)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

    m_angleMotor.resetEncoder(m_angleAbsoluteEncoder.getAbsolutePosition().getValue());
  }

  @Override
  public void periodic() {
    m_drivingMotor.updateInputs(m_drivingMotorInputs);
    m_angleMotor.updateInputs(m_angleMotorInputs);
  }

  public void setDesiredState(SwerveModuleState desiredState) {

    // desiredState.optimize(Rotation2d.fromRotations(m_angleMotor.getPos().in(Rotations)));

    m_drivingMotor.setVelocity(
        RotationsPerSecond.of(
            desiredState.speedMetersPerSecond / Constants.Chassis.DriveMotorConversion));

    m_angleMotor.setPosition(desiredState.angle.getMeasure());

    // if (desiredState.angle.getMeasure() == m_angleMotor.getPos()) {
    //   m_angleMotor.brake();
    // }

    if (desiredState.speedMetersPerSecond == 0.0) {
      m_drivingMotor.brake();
    }
  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(
        m_drivingMotorInputs.velocity.in(RotationsPerSecond)
            * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotorInputs.position));
  }

  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        m_drivingMotorInputs.position.in(Rotations) * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotorInputs.position));
  }
}
