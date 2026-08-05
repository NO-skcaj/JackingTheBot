// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  public void driveRobotRelative(ChassisSpeeds speeds);

  public void setModuleStates(SwerveModuleState[] states);

  public SwerveModuleState[] getModuleStates();

  public SwerveModulePosition[] getModulePositions();

  public ChassisSpeeds getSpeeds();

  public Pose2d getPose();

  public void resetPose(Pose2d newPos);

  public default void resetGyro() {}

  public void periodic();

  public Rotation2d getGyroHeading();
}
