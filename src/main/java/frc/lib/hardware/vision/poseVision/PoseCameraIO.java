// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.vision.poseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.lib.hardware.vision.poseVision.PoseVision.VisionData;
import frc.o2026.Configs;
import frc.o2026.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.photonvision.simulation.PhotonCameraSim;

public interface PoseCameraIO {

  public ArrayList<VisionData> getMeasurements();

  public default PhotonCameraSim getSimCamera() {
    return null;
  }

  public Transform3d getOffset();

  public List<Pose2d> getLastSeenTags();

  public default void addGyroResetter(Consumer<Rotation3d> gyroResetter) {}

  public default void periodic() {}

  public static Pose3d getTagPose(int fiduciary) {

    return Constants.Vision.TagLayout.getTagPose(fiduciary).orElse(new Pose3d());
  }

  public static Matrix<N4, N1> getEstimationStdDevs(Pose2d estimatedPose, int[] targets) {

    // Pose present. Start running Heuristic
    var estStdDevs = Configs.Vision.kSingleTagStdDevs;
    double avgDist = 0;

    if (targets.length == 0) {
      // No tags visible. Default to single-tag std devs
      return Configs.Vision.kSingleTagStdDevs;
    }

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (var tgt : targets) {
      var tagPose = PoseCameraIO.getTagPose(tgt);
      if (tagPose.equals(new Pose3d())) continue;

      avgDist += tagPose.toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    avgDist /= targets.length;

    // Decrease std devs if multiple targets are visible
    if (targets.length > 1) estStdDevs = Configs.Vision.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    // max distance 15 meters
    if (targets.length == 1 && avgDist > 15) {
      return VecBuilder.fill(
          Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      return estStdDevs.times(1 + (Math.pow(avgDist, 2) / 30));
    }
  }
}
