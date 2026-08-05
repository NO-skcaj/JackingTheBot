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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Constants;
import frc.o2026.Robot;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class PoseVision extends SubsystemBase {

  private VisionSystemSim visionSim;

  private Consumer<VisionData> m_poseEstimatorConsumer;

  public List<PoseCameraIO> m_cameras;

  public PoseVision(Consumer<VisionData> poseEstimatorConsumer, PoseCameraIO... cameras) {

    m_poseEstimatorConsumer = poseEstimatorConsumer;
    m_cameras = Arrays.asList(cameras);

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.Vision.TagLayout);

      for (PoseCameraIO m_camera : cameras)
        visionSim.addCamera(m_camera.getSimCamera(), m_camera.getOffset());
    }
  }

  public void addGyroResetter(Consumer<Rotation3d> gyroResetter) {

    for (PoseCameraIO camera : m_cameras) camera.addGyroResetter(gyroResetter);
  }

  public void update() {

    update(null);
  }

  public void update(Pose2d simRealPos) {

    if (simRealPos != null) visionSim.update(simRealPos);

    for (PoseCameraIO camera : m_cameras) {
      for (VisionData data : camera.getMeasurements()) {
        m_poseEstimatorConsumer.accept(data);
      }
    }
  }

  @Override
  public void periodic() {

    Logger.recordOutput(
        "Vision/seenTargets",
        m_cameras.stream()
            .map(PoseCameraIO::getLastSeenTags)
            .filter(poses -> poses != null)
            .filter(poses -> !poses.isEmpty())
            .flatMap(poses -> poses.stream())
            .toList()
            .toArray(Pose2d[]::new));
  }

  public record VisionData(
      Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs, int[] target) {

    public Matrix<N3, N1> get2dStdDevs() {

      return VecBuilder.fill(stdDevs().get(0, 0), stdDevs().get(1, 0), stdDevs().get(3, 0));
    }
  }
}
