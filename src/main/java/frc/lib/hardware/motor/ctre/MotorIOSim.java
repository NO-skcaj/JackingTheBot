// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.ctre;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.SimBattery;
import frc.lib.hardware.motor.MotorConfig;

public class MotorIOSim extends MotorIOTalonFX {

  private final TalonFXSimState m_talonFXSim;

  private final DCMotorSim m_motorSimModel;

  private final double m_gearRatio;

  public MotorIOSim(
      int id, MotorConfig config, boolean isX60, DCMotor gearbox, double moi, double gearRatio) {

    super(id, config);

    m_talonFXSim = m_motor.getSimState();
    m_talonFXSim.setMotorType(isX60 ? MotorType.KrakenX60 : MotorType.KrakenX44);

    m_motorSimModel =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, gearRatio), gearbox);

    m_gearRatio = gearRatio;

    SimBattery.registerDevice(String.valueOf(id), m_motor.getSupplyCurrent().asSupplier());
  }

  @Override
  public void periodic() {

    // set the supply voltage of the TalonFX
    m_talonFXSim.setSupplyVoltage(SimBattery.getSupplyVoltage().in(Volts));

    // get the motor voltage of the TalonFX
    var motorVoltage = m_talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInput(
        MathUtil.clamp(
            motorVoltage.in(Volts),
            -SimBattery.getSupplyVoltage().in(Volts),
            SimBattery.getSupplyVoltage().in(Volts)));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    m_talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(m_gearRatio));
    m_talonFXSim.setRotorVelocity((m_motorSimModel.getAngularVelocityRPM() / 60.0) * m_gearRatio);
  }

  @Override
  public void resetEncoder(Angle angle) {

    super.resetEncoder(angle);

    m_motorSimModel.setAngle(angle.in(Radians));
  }
}
