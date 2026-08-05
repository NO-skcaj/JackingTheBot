// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon implements GyroIO {

  private final Pigeon2 m_gyro;

  public GyroIOPigeon(int id) {

    m_gyro = new Pigeon2(id);
  }

  @Override
  public Rotation3d getGyroRotation() {
    return new Rotation3d(
        m_gyro.getRoll().getValue(),
        m_gyro.getPitch().getValue(),
        m_gyro.getRotation2d().getMeasure());
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() {
    return m_gyro.getAngularVelocityZWorld().getValue();
  }

  @Override
  public void reset() {
    m_gyro.reset();
  }

  @Override
  public void reset(Rotation3d rotation) {
    m_gyro.setYaw(rotation.toRotation2d().getMeasure());
  }
}
