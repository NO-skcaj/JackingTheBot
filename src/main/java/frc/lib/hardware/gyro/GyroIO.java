// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {

  public Rotation3d getGyroRotation();

  public AngularVelocity getGyroAngularVelocity();

  public void reset(Rotation3d rotation);

  public void reset();
}
