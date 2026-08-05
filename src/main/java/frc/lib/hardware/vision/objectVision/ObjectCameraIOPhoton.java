// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.vision.objectVision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.lib.hardware.vision.VisionConfig;
import frc.lib.hardware.vision.objectVision.ObjectVision.ObjectTargetData;
import frc.o2026.Constants;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class ObjectCameraIOPhoton implements ObjectCameraIO {

  private final PhotonCamera m_camera;

  private final VisionConfig m_config;

  public ObjectCameraIOPhoton(VisionConfig config) {

    m_config = config;

    m_camera = new PhotonCamera(m_config.name());
  }

  @Override
  public Set<ObjectTargetData> getObjects() {

    Set<ObjectTargetData> dataSet = new HashSet<>();

    for (var result : m_camera.getAllUnreadResults()) {
      for (var target : result.getTargets()) {

        // --- Range estimation using known geometry ---
        // PhotonVision OD does NOT provide a reliable 3D translation in bestCameraToTarget.
        // We instead compute range from the camera-height / target-pitch trig relationship,
        // using the known height of the Fuel ball center above the floor.

        double rangeMeters =
            PhotonUtils.calculateDistanceToTargetMeters(
                m_config.offset().getTranslation().getZ(),
                Constants.Field.CoralDiameter.in(Meters) / 2.0,
                -m_config.offset().getRotation().getMeasureY().in(Radians),
                Degrees.of(target.getPitch()).in(Radians));

        Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw());

        // Camera-relative (X forward, Y left). This is a pure flat-ground approximation —
        Translation3d cameraToTarget =
            new Translation3d(
                rangeMeters * yaw.getCos(),
                rangeMeters * yaw.getSin(),
                (Constants.Field.CoralDiameter.in(Meters) / 2.0)
                    - m_config.offset().getTranslation().getZ());

        Translation3d robotToTarget =
            cameraToTarget
                .rotateBy(m_config.offset().getRotation())
                .plus(m_config.offset().getTranslation());

        dataSet.add(new ObjectTargetData(target.objDetectId, target.objDetectConf, robotToTarget));
      }
    }

    return dataSet;
  }

  @Override
  public Optional<Angle> getRotToBestObject() {

    var results = m_camera.getAllUnreadResults();
    if (results.size() == 0) return Optional.empty();

    PhotonPipelineResult objectResult = results.get(0);
    if (objectResult != null) {
      if (objectResult.hasTargets()) {

        Logger.recordOutput("odYaw", objectResult.getBestTarget().getYaw());
        return Optional.of(Degrees.of(objectResult.getBestTarget().getYaw()));
      }
    }

    return Optional.empty();
  }

  public Transform3d getRobotToCamera() {
    return m_config.offset();
  }
}
