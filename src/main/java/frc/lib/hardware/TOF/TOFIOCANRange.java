// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.TOF;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

public class TOFIOCANRange implements TOFIO {

  private CANrange m_sensor;
  private StatusSignal<Boolean> m_isDetected;

  public TOFIOCANRange(int id) {

    m_sensor = new CANrange(id);

    m_isDetected = m_sensor.getIsDetected();
  }

  public TOFIOCANRange(int id, TOFConfig config) {

    this(id);

    config(config);
  }

  @Override
  public void config(TOFConfig config) {

    m_sensor
        .getConfigurator()
        .apply(
            new ProximityParamsConfigs()
                .withProximityThreshold(config.getMinDistance())
                .withMinSignalStrengthForValidMeasurement(config.getMinSignalStrength()));
  }

  // TODO: CHANGE TO LoggedInputs PATTERN
  @Override
  public boolean isDetected() {

    return m_isDetected.getValue();
  }
}
