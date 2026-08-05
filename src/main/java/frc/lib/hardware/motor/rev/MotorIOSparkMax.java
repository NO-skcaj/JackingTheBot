// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.hardware.SimBattery;
import frc.lib.hardware.motor.MotorConfig;
import frc.lib.hardware.motor.MotorIO;
import org.littletonrobotics.junction.Logger;

public final class MotorIOSparkMax implements MotorIO {

  private SparkMax m_motor;
  private SparkMaxConfig m_motorConfig;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_controller;

  private double m_lastRef = 0.0;

  private MotorConfig m_config = new MotorConfig();

  public MotorIOSparkMax(int id) {

    m_motor = new SparkMax(id, MotorType.kBrushless);
    m_motorConfig = new SparkMaxConfig();
    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getClosedLoopController();
  }

  public MotorIOSparkMax(int id, MotorConfig config) {

    this(id);

    config(config);
  }

  public void config(MotorConfig config) {

    m_config = config;

    m_motorConfig
        .closedLoop
        .pid(config.getP(), config.getI(), config.getD())
        .positionWrappingEnabled(config.isContinuousWrap());

    m_motorConfig
        .inverted(config.isInverted())
        .idleMode(config.isBrakeMode() ? IdleMode.kBrake : IdleMode.kCoast);

    m_motorConfig.smartCurrentLimit((int) config.getStatorCurrent().in(Amps));

    m_motorConfig
        .encoder
        .positionConversionFactor(1.0 / config.getSensorToMechanismRatio())
        .velocityConversionFactor((1.0 / config.getSensorToMechanismRatio()) / 60.0);

    m_motorConfig
        .absoluteEncoder
        .positionConversionFactor((1 / config.getSensorToMechanismRatio()))
        .velocityConversionFactor((1 / config.getSensorToMechanismRatio()) / 60.0);

    // Write the configuration to the motor controller
    m_motor.configure(
        m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  @Override
  public int getId() {

    return m_motor.getDeviceId();
  }

  public void follow(int id, boolean inverted) {

    m_motorConfig.follow(id, inverted);

    m_motor.configure(
        m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void brake() {

    // m_motor.set(0.0);
    m_lastRef = 0.0;

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void setPosition(Angle angle) {

    m_controller.setSetpoint(angle.in(Rotations), ControlType.kPosition);

    m_lastRef = angle.in(Rotations);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void setVelocity(AngularVelocity angleVel) {

    m_controller.setSetpoint(angleVel.in(RotationsPerSecond), ControlType.kVelocity);

    m_lastRef = angleVel.in(RotationsPerSecond);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void setVoltage(Voltage volts) {

    m_controller.setSetpoint(volts.in(Volts), ControlType.kVoltage);

    m_lastRef = volts.in(Volts);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void resetEncoder(Angle angle) {

    m_encoder.setPosition(angle.in(Rotations));

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    
    inputs.velocity = RotationsPerSecond.of(m_encoder.getVelocity());
    inputs.position = Rotations.of(m_encoder.getPosition());
    inputs.rawPosition = Rotations.of(m_encoder.getPosition() / m_config.getSensorToMechanismRatio());
    inputs.appliedVolts = Volts.of(m_motor.getAppliedOutput() * (RobotBase.isReal() ? m_motor.getBusVoltage() : SimBattery.getSupplyVoltage().in(Volts)));
    inputs.statorCurrent = Amps.of(m_motor.getOutputCurrent());
    inputs.temperature = Celsius.of(m_motor.getMotorTemperature());
    inputs.lastReference = m_lastRef;
  }
}
