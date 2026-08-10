// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.TOF;

// Basic distance sensor
public interface TOFIO {

  public void config(TOFConfig config);

  // TODO: CHANGE TO LoggedInputs PATTERN
  public boolean isDetected();
}
