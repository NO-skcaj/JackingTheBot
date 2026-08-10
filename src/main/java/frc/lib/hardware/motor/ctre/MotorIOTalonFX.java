// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.motor.MotorConfig;
import frc.lib.hardware.motor.MotorIO;
import org.littletonrobotics.junction.Logger;

public class MotorIOTalonFX implements MotorIO {

  final TalonFX m_motor;

  final StatusSignal<AngularVelocity> m_velocity;
  final StatusSignal<Angle> m_position;
  final StatusSignal<Angle> m_rawPosition;
  final StatusSignal<Voltage> m_appliedVolts;
  final StatusSignal<Temperature> m_temperature;
  final StatusSignal<Current> m_currentStator;
  final StatusSignal<Current> m_currentSupply;

  private double m_lastRef = 0.0;

  public MotorIOTalonFX(int id) {

    m_motor = new TalonFX(id);

    m_velocity = m_motor.getVelocity();
    m_position = m_motor.getPosition();
    m_rawPosition = m_motor.getRotorPosition();
    m_appliedVolts = m_motor.getMotorVoltage();
    m_currentStator = m_motor.getStatorCurrent();
    m_currentSupply = m_motor.getSupplyCurrent();
    m_temperature = m_motor.getDeviceTemp();

    OrchestraOrchestrator.addInstrument(m_motor);
  }

  public MotorIOTalonFX(int id, MotorConfig config) {

    this(id);

    config(config);
  }

  public void config(MotorConfig config) {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrent().in(Amps);
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.getSupplyCurrent().in(Amps) != 0;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrent().in(Amps);
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.getStatorCurrent().in(Amps) != 0;

    talonConfig.MotorOutput.Inverted =
        config.isInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode =
        config.isBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    talonConfig.Feedback.SensorToMechanismRatio = config.getSensorToMechanismRatio();

    talonConfig.ClosedLoopGeneral.ContinuousWrap = config.isContinuousWrap();

    talonConfig.Slot0.kP = config.getP();
    talonConfig.Slot0.kI = config.getI();
    talonConfig.Slot0.kD = config.getD();
    talonConfig.Slot0.kS = config.getS();
    talonConfig.Slot0.kV = config.getV();
    talonConfig.Slot0.kA = config.getA();

    talonConfig.MotionMagic.MotionMagicCruiseVelocity =
        config.getVelocityLimit().in(RotationsPerSecond);
    talonConfig.MotionMagic.MotionMagicAcceleration =
        config.getAccelerationLimit().in(RotationsPerSecondPerSecond);

    talonConfig.Audio.AllowMusicDurDisable = true;

    var status = m_motor.getConfigurator().apply(talonConfig);

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public int getId() {

    return m_motor.getDeviceID();
  }

  @Override
  public void setVelocity(AngularVelocity angleVel) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVelocityVoltage(angleVel));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    m_lastRef = m_motor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setPosition(Angle angle) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVoltage(angle));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    m_lastRef = m_motor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setVoltage(Voltage volts) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new VoltageOut(volts));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    m_lastRef = volts.in(Volts);
  }

  @Override
  public void follow(int id, boolean inverted) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status =
        m_motor.setControl(
            new Follower(id, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void brake() {

    var status = m_motor.setControl(new NeutralOut());
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    OrchestraOrchestrator.addInstrument(m_motor);

    m_lastRef = 0.0;
  }

  @Override
  public void resetEncoder(Angle angle) {

    var status = m_motor.setPosition(angle);
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void updateInputs(MotorInputs inputs) {

    inputs.velocity = m_velocity.refresh().getValue();
    inputs.position = m_position.refresh().getValue();
    inputs.rawPosition = m_rawPosition.refresh().getValue();
    inputs.appliedVolts = m_appliedVolts.refresh().getValue();
    inputs.statorCurrent = m_currentStator.refresh().getValue();
    inputs.temperature = m_temperature.refresh().getValue();
    inputs.lastReference = m_lastRef;
  }
}
