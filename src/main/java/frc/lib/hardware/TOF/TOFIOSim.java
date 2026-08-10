// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.TOF;

import java.util.function.Supplier;

public class TOFIOSim implements TOFIO {

  private Supplier<Boolean> m_supplier;

  public TOFIOSim(Supplier<Boolean> supplier) {
    m_supplier = supplier;
  }

  @Override
  public void config(TOFConfig config) {}

  @Override
  public boolean isDetected() {

    return m_supplier.get();
  }
}
