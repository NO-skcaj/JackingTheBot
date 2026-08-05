// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.vision.objectVision;

import edu.wpi.first.units.measure.Angle;
import frc.lib.hardware.vision.objectVision.ObjectVision.ObjectTargetData;

import java.util.Optional;
import java.util.Set;

public interface ObjectCameraIO {

  public Optional<Angle> getRotToBestObject();

  public Set<ObjectTargetData> getObjects();
}
