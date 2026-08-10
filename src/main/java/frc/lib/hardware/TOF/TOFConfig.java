// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.TOF;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import lombok.Getter;
import lombok.With;

@With
public class TOFConfig {

  @Getter private double minSignalStrength;
  @Getter private Distance minDistance;

  public TOFConfig() {
    minSignalStrength = 0.0;
    minDistance = Meters.of(0.0);
  }

  public TOFConfig(double minSignalStrength, Distance minDistance) {

    this.minSignalStrength = minSignalStrength;
    this.minDistance = minDistance;
  }
}
