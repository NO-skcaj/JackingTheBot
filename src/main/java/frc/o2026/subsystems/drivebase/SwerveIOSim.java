// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.hawkLib.hardware.vision.poseVision.PoseCameraIO;
import frc.hawkLib.hardware.vision.poseVision.PoseVision;
import frc.hawkLib.reefscape.ReefscapeIntakeUtil;
import frc.hawkLib.sim.SelfControlledSwerveDriveSimulation;
import frc.hawkLib.sim.SwerveDriveSimulation;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveIOSim implements SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  // Create and configure a drivetrain simulation configuration
  private static DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Pounds.of(130.0))
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60(1), // Drive motor
                  DCMotor.getNEO(1), // Steer motor
                  COTS.WHEELS.BLUE_NITRILE_TREAD.cof,
                  2)) // L3 Gear ratio
          .withTrackLengthTrackWidth(
              Constants.Chassis.WheelBaseMeters, Constants.Chassis.TrackWidthMeters)
          .withBumperSize(
              Constants.Chassis.WheelBaseMeters.plus(Inches.of(4.5).times(2.0)),
              Constants.Chassis.TrackWidthMeters.plus(Inches.of(4.5).times(2.0)));

  private static SelfControlledSwerveDriveSimulation m_swerve =
      new SelfControlledSwerveDriveSimulation(
          new SwerveDriveSimulation(
              driveTrainSimulationConfig, new Pose2d(2, 2, new Rotation2d(Math.PI))));

  private final IntakeSimulation m_intake =
      IntakeSimulation.OverTheBumperIntake(
          "Coral",
          m_swerve.getDriveTrainSimulation(),
          Inches.of(20.0), // Width
          Inches.of(12.0),
          IntakeSide.FRONT,
          1);

  private PoseVision m_vision;

  private boolean m_xMode = false;

  public SwerveIOSim(PoseCameraIO... cameras) {

    m_vision =
        new PoseVision(
            (data) ->
                m_swerve.addVisionEstimation(
                    data.visionMeasurement().toPose2d(),
                    data.timestampSeconds(),
                    data.get2dStdDevs()),
            cameras);

    m_vision.addGyroResetter(newRot -> m_swerve.resetGyro(newRot.toRotation2d()));

    RobotState.getSimArena().addDriveTrainSimulation(m_swerve.getDriveTrainSimulation());
  }

  @Override
  public Pose2d getPose() {

    // return m_swerve.getOdometryEstimatedPose();
    return m_swerve.getOdometryEstimatedPose();
  }

  @Override
  public Rotation2d getGyroHeading() {
    return getPose().getRotation();
  }

  @Override
  public void driveRobotRelative(ChassisSpeeds speeds) {

    // If the chassis is in x mode, than stay in x mode, ignoring the desired speeds
    if (m_xMode) {
      // Set the module states to x mode
      setModuleStates((SwerveModuleState[]) Constants.Chassis.XishStates.toArray());

      // Save the desired speeds for logging later
      return;
    }

    // Set the desired state for each swerve module
    m_swerve.runChassisSpeeds(speeds, new Translation2d(), false, true);

    Logger.recordOutput("Swerve/XMode", m_xMode);
  }

  @Override
  public void setModuleStates(SwerveModuleState[] inputs) {

    m_swerve.runChassisSpeeds(
        Constants.Chassis.Kinematics.toChassisSpeeds(
            new SwerveModuleState[] {inputs[0], inputs[1], inputs[2], inputs[3]}),
        new Translation2d(),
        false,
        true);
  }

  @Override
  public void periodic() {

    m_vision.update(m_swerve.getActualPoseInSimulationWorld());

    m_swerve.periodic();

    if (RobotState.isSimIntaking() && !RobotState.isHasCoral()) {
      m_intake.startIntake();
      if (m_intake.obtainGamePieceFromIntake() || ReefscapeIntakeUtil.hasNewCoralFromCollector()) {
        RobotState.setHasCoral(true);
      }
    } else {
      m_intake.stopIntake();
    }

    Pose2d simPose = m_swerve.getActualPoseInSimulationWorld();

    RobotState.setSimRealPose(new Pose3d(simPose));
    RobotState.setSimSpeeds(m_swerve.getActualSpeedsFieldRelative());

    Logger.recordOutput("Sim/Pose3d", simPose);
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    return m_swerve.getMeasuredStates();
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    return m_swerve.getLatestModulePositions();
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return Constants.Chassis.Kinematics.toChassisSpeeds(getModuleStates());
  }

  public boolean getIsXMode() {
    return m_xMode;
  }

  public void setIsXMode(boolean xMode) {
    m_xMode = xMode;
  }

  @Override
  public void resetPose(Pose2d newPos) {

    m_swerve.setSimulationWorldPose(newPos);
    m_swerve.resetOdometry(newPos);
  }
}
