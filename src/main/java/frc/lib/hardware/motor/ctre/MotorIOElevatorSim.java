// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.hardware.SimBattery;
import frc.lib.hardware.motor.MotorConfig;

public class MotorIOElevatorSim extends MotorIOTalonFX {

  private final TalonFXSimState m_talonFXSim;

  private final ElevatorSim m_motorSimModel;

  private double m_gearRatio;
  private Distance m_drumWidth;

  public MotorIOElevatorSim(
      int id, 
      MotorConfig config, 
      boolean isX60, 
      double moi, 
      int numMotorsInSystem, 
      double gearRatio,
      Distance minHeight,
      Distance maxHeight,
      Mass carriageWeight,
      Distance drumWidth) {

    super(id, config);

    m_drumWidth = drumWidth;

    m_talonFXSim = m_motor.getSimState();
    m_talonFXSim.setMotorType(isX60 ? MotorType.KrakenX60 : MotorType.KrakenX44);
    
    DCMotor motor = isX60 ? DCMotor.getKrakenX60(numMotorsInSystem) : DCMotor.getKrakenX44(numMotorsInSystem);

    m_motorSimModel =
        new ElevatorSim(
            motor,
            gearRatio,
            carriageWeight.in(Kilogram),
            drumWidth.in(Meters),
            minHeight.in(Meters),
            maxHeight.in(Meters),
            true,
0.0);

    m_gearRatio = gearRatio;

    SimBattery.registerDevice(String.valueOf(id), () -> Amps.of(m_motorSimModel.getCurrentDrawAmps()));
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
    m_talonFXSim.setRotorVelocity(
        (m_motorSimModel.getVelocityMetersPerSecond() / m_drumWidth.in(Meters)));
  }

  @Override
  public void updateInputs(MotorInputs inputs) {

    super.updateInputs(inputs);

    inputs.statorCurrent = Amps.of(m_motorSimModel.getCurrentDrawAmps());
    inputs.appliedVolts = Volts.of(m_motorSimModel.getInput().get(0, 0));
  }

  @Override
  public void resetEncoder(Angle angle) {

    super.resetEncoder(angle);
  }
}
