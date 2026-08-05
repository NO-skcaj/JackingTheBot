// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;

// MXP NavX btw
public class GyroIONavX implements GyroIO {

  private AHRS m_gyro;

  public GyroIONavX() {

    m_gyro = new AHRS(NavXComType.kMXP_SPI);
  }

  @Override
  public Rotation3d getGyroRotation() {

    return new Rotation3d(
        Degrees.of(m_gyro.getRoll()), Degrees.of(m_gyro.getPitch()), Degrees.of(-m_gyro.getYaw()));
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() {

    return DegreesPerSecond.of(-m_gyro.getRate());
  }

  @Override
  public void reset() {

    m_gyro.reset();
  }

  @Override
  public void reset(Rotation3d rotation) {

    m_gyro.reset();
  }
}
