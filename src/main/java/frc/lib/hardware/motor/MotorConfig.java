// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import lombok.Getter;
import lombok.With;

@With
public class MotorConfig {

  @Getter Current supplyCurrent;
  @Getter Current statorCurrent;
  @Getter boolean inverted;
  @Getter boolean brakeMode;
  @Getter boolean continuousWrap;
  @Getter double P;
  @Getter double I;
  @Getter double D;
  @Getter double S;
  @Getter double V;
  @Getter double A;
  @Getter AngularVelocity velocityLimit;
  @Getter AngularAcceleration accelerationLimit;
  @Getter double sensorToMechanismRatio;

  public MotorConfig() {
    supplyCurrent = Amps.of(70.0);
    statorCurrent = Amps.of(120.0);
    inverted = false;
    brakeMode = true;
    continuousWrap = false;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    S = 0.0;
    V = 0.0;
    A = 0.0;
    velocityLimit = RotationsPerSecond.of(600.0);
    accelerationLimit = RotationsPerSecondPerSecond.of(6000.0);
    sensorToMechanismRatio = 1.0;
  }

  public MotorConfig(
      Current supplyCurrent,
      Current statorCurrent,
      boolean inverted,
      boolean brakeMode,
      boolean continuousWrap,
      double P,
      double I,
      double D,
      double S,
      double V,
      double A,
      AngularVelocity velocityLimit,
      AngularAcceleration accelerationLimit,
      double sensorToMechanismRatio) {

    this.supplyCurrent = supplyCurrent;
    this.statorCurrent = statorCurrent;
    this.inverted = inverted;
    this.brakeMode = brakeMode;
    this.continuousWrap = continuousWrap;
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.velocityLimit = velocityLimit;
    this.accelerationLimit = accelerationLimit;
    this.sensorToMechanismRatio = sensorToMechanismRatio;
  }
}
