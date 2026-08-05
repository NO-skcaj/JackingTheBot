// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {

  public default void periodic() {}

  public void config(MotorConfig config);

  public int getId();

  public void follow(int id, boolean inverted);

  public void brake();

  public void setPosition(Angle angle);

  public void setVelocity(AngularVelocity angleVel);
  
  public void setVoltage(Voltage volts);

  public void resetEncoder(Angle angle);

  public void updateInputs(MotorInputs inputs);

  // NO @AutoLog because it doesn't work in my static analysis (:
  public static class MotorInputs implements LoggableInputs {

    public AngularVelocity velocity;
    public Angle position;
    public Angle rawPosition;
    public Voltage appliedVolts;
    public Current statorCurrent;
    public Temperature temperature;
    public double lastReference;

    @Override
    public void toLog(LogTable table) {
      table.put("velocity", velocity);
      table.put("position", position);
      table.put("rawPosition", rawPosition);
      table.put("appliedVolts", appliedVolts);
      table.put("statorCurrent", statorCurrent);
      table.put("temperature", temperature);
      table.put("lastReference", lastReference);
    }
    
    @Override
    public void fromLog(LogTable table) {
      velocity = table.get("velocity", velocity);
      position = table.get("position", position);
      rawPosition = table.get("rawPosition", rawPosition);
      appliedVolts = table.get("appliedVolts", appliedVolts);
      statorCurrent = table.get("statorCurrent", statorCurrent);
      temperature = table.get("temperature", temperature);
      lastReference = table.get("lastReference", lastReference);
    }
  }
}
