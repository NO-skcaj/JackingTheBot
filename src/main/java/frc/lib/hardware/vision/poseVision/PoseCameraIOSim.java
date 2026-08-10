// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.vision.poseVision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.vision.VisionConfig;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

// The difference between the photonvision hardware and photonvision sim is very small
// We use inheritence, despite it not being good practice, due to the simplicity of the
// sim implementation.
public class PoseCameraIOSim extends PoseCameraIOPhoton {

  private int m_xRes = 1200;
  private int m_yRes = 800;
  private double m_fov = 70;
  private double m_fps = 15;

  private VisionSystemSim m_visionSim;

  private PhotonCameraSim m_simCamera;

  public PoseCameraIOSim(VisionConfig config) {

    super(config);

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(m_xRes, m_yRes, Rotation2d.fromDegrees(m_fov));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(m_fps);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    m_simCamera = new PhotonCameraSim(m_camera, cameraProp, Constants.Vision.TagLayout);
    m_simCamera.enableDrawWireframe(false);
  }

  @Override
  public PhotonCameraSim getSimCamera() {

    return m_simCamera;
  }

  @Override
  public void periodic() {

    m_visionSim.update(RobotState.getSimRealPose());
  }
}
