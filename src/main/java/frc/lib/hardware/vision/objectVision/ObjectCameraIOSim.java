// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.vision.objectVision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.hardware.vision.VisionConfig;
import frc.lib.hardware.vision.objectVision.ObjectVision.ObjectTargetData;
import frc.o2026.RobotState;
import java.util.Comparator;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class ObjectCameraIOSim implements ObjectCameraIO {

  private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75),
      CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(45);

  private VisionConfig m_config;

  private Distance SeeableDist = Meters.of(5.0);

  public ObjectCameraIOSim(VisionConfig config) {

    m_config = config;
  }

  @Override
  public Set<ObjectTargetData> getObjects() {
    Pose3d robotPose = RobotState.getSimRealPose();

    return RobotState.getSimArena().gamePiecesOnField().stream()
        .map(GamePieceOnFieldSimulation::getPose3d)
        .map(Pose3d::getTranslation)
        .filter(
            translation ->
                robotPose.getTranslation().getDistance(translation) < SeeableDist.in(Meters))
        .map(
            translation -> {
              // translation is field-relative. Convert to robot-relative:
              Translation2d robotRelative2d =
                  translation
                      .toTranslation2d()
                      .minus(robotPose.getTranslation().toTranslation2d())
                      .rotateBy(robotPose.getRotation().toRotation2d().unaryMinus());
              double relativeZ = translation.getZ() - robotPose.getZ();
              return new Translation3d(robotRelative2d.getX(), robotRelative2d.getY(), relativeZ);
            })
        .filter(
            robotToTarget -> {
              // Convert robot-relative target position to camera-relative:
              Translation3d cameraToTarget =
                  robotToTarget
                      .minus(m_config.offset().getTranslation())
                      .rotateBy(m_config.offset().getRotation().unaryMinus());

              // Target is in front of camera (X > 0)
              if (cameraToTarget.getX() <= 0) return false;

              // Check horizontal FOV
              double horizontalAngle = Math.atan2(cameraToTarget.getY(), cameraToTarget.getX());
              if (Math.abs(horizontalAngle) > CAMERA_HORIZONTAL_FOV.getRadians() / 2.0)
                return false;

              // Check vertical FOV
              double verticalAngle = Math.atan2(cameraToTarget.getZ(), cameraToTarget.getX());
              if (Math.abs(verticalAngle) > CAMERA_VERTICAL_FOV.getRadians() / 2.0) return false;

              return true;
            })
        .map(robotToTarget -> new ObjectTargetData(0, 1.0, robotToTarget))
        .collect(Collectors.toSet());
  }

  @Override
  public Optional<Angle> getRotToBestObject() {

    Optional<ObjectTargetData> closestObject =
        getObjects().stream()
            .min(
                Comparator.comparingDouble(data -> data.translation().toTranslation2d().getNorm()));

    if (closestObject.isEmpty()) return Optional.empty();

    // Convert the robot-relative target position to camera-relative, and extract its yaw angle
    Translation3d robotToTarget = closestObject.get().translation();
    Translation3d cameraToTarget =
        robotToTarget
            .minus(m_config.offset().getTranslation())
            .rotateBy(m_config.offset().getRotation().unaryMinus());

    return Optional.of(cameraToTarget.toTranslation2d().getAngle().getMeasure());
  }
}
