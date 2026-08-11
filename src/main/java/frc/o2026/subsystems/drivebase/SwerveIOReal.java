// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.shared.hardware.gyro.GyroIO;
import frc.shared.hardware.gyro.GyroIONavX;
import frc.shared.hardware.vision.poseVision.PoseCameraIO;
import frc.shared.hardware.vision.poseVision.PoseVision;
import frc.o2026.Configs;
import frc.o2026.Constants;
import org.littletonrobotics.junction.Logger;

/// @brief Chassis subsystem for swerve drive control
///
///       Red                      <----- Zero Angle                       Blue
///                            <--- 0 degrees    180 degrees ---->     X  <-----
///   ---  +-------------------------------------------------------------------+ (0, 0)
///    ^   |                7  6              |             17 28           29 |
///    |   |                                  |                             30 |  |
///    |   |                                  |                                |  |
///    |   |                                  |                                |  V
///    |   |                                  |                                |
///    |   |                8  5              |             18 27              |  Y
/// 8.07 m | 16          9       4            |          19       26        31 |
///    |   | 15         10       3            |          20       25        32 |
///    |   |               11  2              |             21 24              |
///    |   |                                  |                                |
///    |   |                                  |                                |
///    |   | 14                               |                                |
///    V   | 13            12  1              |             22 23              |
///   ---  +-------------------------------------------------------------------+
///        |<----------------------------- 16.56 m --------------------------->|
///                                       Top View
public class SwerveIOReal implements SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  private SwerveModule m_frSwerveModules;
  private SwerveModule m_flSwerveModules;
  private SwerveModule m_brSwerveModules;
  private SwerveModule m_blSwerveModules;

  private SwerveDrivePoseEstimator3d m_estimator;

  ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds(0, 0, 0);

  boolean m_xMode = false;

  private GyroIO m_gyroIO;
  private PoseVision m_vision;

  private GyroIO m_GyroIO2 = new GyroIONavX();

  public SwerveIOReal(SwerveModule fr, SwerveModule fl, 
                      SwerveModule br, SwerveModule bl, 
                      GyroIO gyroIO, 
                      PoseCameraIO... cameras) {

    m_gyroIO = gyroIO;

    m_vision =
        new PoseVision(
            (data) ->
                m_estimator.addVisionMeasurement(
                    data.visionMeasurement(), data.timestampSeconds(), data.stdDevs()),
            cameras);

    m_vision.addGyroResetter(m_gyroIO::reset);

    m_estimator =
        new SwerveDrivePoseEstimator3d(
            Constants.Chassis.Kinematics,
            m_gyroIO.getGyroRotation(), // Initial gyro angle
            getModulePositions(),
            new Pose3d(14.0, 7.0, 0.0, new Rotation3d(Rotation2d.k180deg)) // Initial pose
            );
  }

  @Override
  public void periodic() {

    m_vision.update();

    m_estimator.update(m_gyroIO.getGyroRotation(), getModulePositions());

    Logger.recordOutput("Navx-Output ", m_GyroIO2.getGyroRotation().getMeasureZ().in(Degrees));
    Logger.recordOutput("Pidgeon-Output ", m_gyroIO.getGyroRotation().getMeasureZ().in(Degrees));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {

    if (m_xMode) {
      // Set the module states to x mode
      setModuleStates(Constants.Chassis.XishStates.toArray(new SwerveModuleState[0]));

      // Save the desired speeds for logging later
      return;
    }

    // Save the desired states for use and logging later
    SwerveModuleState[] desiredStates = Constants.Chassis.Kinematics.toSwerveModuleStates(speeds);

    // Set the desired state for each swerve module
    setModuleStates(desiredStates);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // Set the desired state for each swerve module
    m_flSwerveModules.setDesiredState(states[0]);
    m_frSwerveModules.setDesiredState(states[1]);
    m_blSwerveModules.setDesiredState(states[2]);
    m_brSwerveModules.setDesiredState(states[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      m_flSwerveModules.getState(),
      m_frSwerveModules.getState(),
      m_blSwerveModules.getState(),
      m_brSwerveModules.getState()
    };

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      m_flSwerveModules.getPosition(),
      m_frSwerveModules.getPosition(),
      m_blSwerveModules.getPosition(),
      m_brSwerveModules.getPosition()
    };

    return positions;
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Chassis.Kinematics.toChassisSpeeds(getModuleStates());
  }

  @Override
  public Pose2d getPose() {

    return m_estimator.getEstimatedPosition().toPose2d();
  }

  @Override
  public void resetPose(Pose2d newPos) {

    m_estimator.resetPose(new Pose3d(newPos));
  }

  @Override
  public void resetGyro() {

    m_gyroIO.reset();
    m_GyroIO2.reset();

    m_estimator.resetPose(
        new Pose3d(m_estimator.getEstimatedPosition().getTranslation(), new Rotation3d()));
  }

  @Override
  public Rotation2d getGyroHeading() {

    return m_gyroIO.getGyroRotation().toRotation2d();
  }
}
