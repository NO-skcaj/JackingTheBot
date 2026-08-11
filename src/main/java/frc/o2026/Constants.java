// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.shared.hardware.vision.VisionConfig;
import java.util.ArrayList;
import java.util.List;

public final class Constants {

  public static final class Field {

    /// *** Field Dimensions *** ///
    public static final double FieldLengthMeters = Units.inchesToMeters(652.11); // 16.56 meters
    public static final double FieldWidthMeters = Units.inchesToMeters(317.69); //  8.07 meters

    public static final Distance CoralDiameter = Inches.of(4.0);

    public static final Translation2d BlueHub =
        new Translation2d(Inches.of(275.0), Inches.of(179.0));
  }

  public static final class Vision {

    public static final VisionConfig BackCamConfig =
        new VisionConfig(
            "RealCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(-15.0), // forwards
                    Inches.of(0.0), // right
                    Inches.of(10.0)), // up
                new Rotation3d(
                    Degrees.of(0.0), // roll
                    Degrees.of(-10.0), // pitch
                    Degrees.of(180.0)))); // yaw

    public static final VisionConfig LimelightOfDoomAndDespair =
        new VisionConfig(
            "limelight-athreeg",
            new Transform3d(
                new Translation3d(
                    Inches.of(0.0), // forwards
                    Inches.of(-15.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(90.0), // roll
                    Degrees.of(0.0), // pitch
                    Degrees.of(90.0)))); // yaw

    public static final VisionConfig FrontCamConfig =
        new VisionConfig(
            "BadCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(15.0), // forwards
                    Inches.of(-3.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(90.0), // roll
                    Degrees.of(-45.0), // pitch
                    Degrees.of(0.0)))); // yaw

    public static final VisionConfig WebCam =
        new VisionConfig(
            "LightCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(-1.0), // forwards
                    Inches.of(15.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(0.0), // roll
                    Degrees.of(-20.0), // pitch
                    Degrees.of(-90.0)))); // yaw

    public static AprilTagFieldLayout TagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  public static final class Superstructure {

    public static final Distance CoralScoreXOffset = Inches.of(22.5);
    public static final Distance CoralScoreYOffset = Inches.of(-6.467946);

    public static final double ElevatorGearRatio = (12.0 / 1.0) * (64.0 / 18.0);
    public static final double ElevatorSpoolRadiusMeters = Units.inchesToMeters(0.758);
    public static final double ElevatorMetersToRotations =
        1 / (ElevatorSpoolRadiusMeters * 2.0 * Math.PI);

    public static record ArmPosition(Angle pivot, Distance elevator, Angle wrist) {}

    public static final ArmPosition Starting =
        new ArmPosition(Degrees.of(45.480), Inches.of(0.0), Degrees.of(27.315));

    public static final ArmPosition Home =
        new ArmPosition(Degrees.of(0.0), Inches.of(0.0), Degrees.of(53.811));

    public static final ArmPosition GroundIntake =
        new ArmPosition(Degrees.of(0.0), Inches.of(0.0), Degrees.of(176.833952));

    public static final ArmPosition L1F =
        new ArmPosition(Degrees.of(43.448), Inches.of(0.0), Degrees.of(66.917056));

    public static final ArmPosition L2F =
        new ArmPosition(Degrees.of(35.04), Inches.of(6.225), Degrees.of(79.288));

    public static final ArmPosition L3F =
        new ArmPosition(Degrees.of(51.479), Inches.of(15.462), Degrees.of(95.726488));

    public static final ArmPosition L4F =
        new ArmPosition(Degrees.of(64.443), Inches.of(34.625), Degrees.of(108.690422));

    public static final ArmPosition L2B =
        new ArmPosition(Degrees.of(106.562), Inches.of(0.334246), Degrees.of(38.412369));

    public static final ArmPosition L3B =
        new ArmPosition(Degrees.of(100.386), Inches.of(12.605856), Degrees.of(44.896124));

    public static final ArmPosition L4B =
        new ArmPosition(Degrees.of(94.914), Inches.of(34.625), Degrees.of(49.424149));

    // TODO: test or research reasonable voltages
    public static final Voltage IntakingCoralVolt = Volts.of(5.0);
    public static final Voltage ScoringCoralVolt = Volts.of(7.0);
    public static final Voltage HoldingCoralVolt = Volts.of(0.0);
    public static final Voltage IntakingAlgaeVolt = Volts.of(8.0);
    public static final Voltage ScoringAlgaeVolt = Volts.of(5.0);
    public static final Voltage HoldingAlgaeVolt = Volts.of(7.0);
  }

  public static final class Chassis {

    // NOTE: The absolute encoder range is 0.5 to -0.5
    // These are the absolute encoder values that correspond to the wheels facing "forward"
    public static final Angle FrontRightForwardsAngle = Rotations.of(-0.1396);
    public static final Angle FrontLeftForwardsAngle = Rotations.of(-0.4800);
    public static final Angle BackRightForwardsAngle = Rotations.of(0.05908);
    public static final Angle BackLeftForwardsAngle = Rotations.of(0.08325);

    public static final Distance WheelBaseMeters = Inches.of(30.0);
    public static final Distance TrackWidthMeters = Inches.of(30.0);

    public static final double DriveMotorReduction = 6.75;
    public static final Distance WheelDiameter = Meters.of(0.098022); // meters
    public static final Distance WheelCircumference = WheelDiameter.times(Math.PI);
    public static final double DriveMotorConversion =
        WheelCircumference.div(DriveMotorReduction).in(Meters); // Meters per motor turn

    public static final ArrayList<SwerveModuleState> XishStates =
        new ArrayList<>(
            List.of(
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), // FL
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), // FR
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), // BL
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)) // BR
                ));

    public static final Translation2d[] ModulePositions = {
      new Translation2d(WheelBaseMeters.div(2.0), TrackWidthMeters.div(2.0)), // Front Left
      new Translation2d(
          WheelBaseMeters.div(2.0), TrackWidthMeters.div(2.0).times(-1.0)), // Front Right
      new Translation2d(
          WheelBaseMeters.div(2.0).times(-1.0), TrackWidthMeters.div(2.0)), // Back Left
      new Translation2d(
          WheelBaseMeters.div(2.0).times(-1.0), TrackWidthMeters.div(2.0).times(-1.0)) // Back Right
    };

    public static final SwerveDriveKinematics Kinematics =
        new SwerveDriveKinematics(ModulePositions);
  }

  public static final class CanIds {

    public static final int FrontLeftDriveId = 31; // Kraken X60
    public static final int FrontLeftTurnId = 32; // Kraken X44 / NEO v1.2
    public static final int FrontLeftEncoderId = 33; // CANCoder

    public static final int FrontRightDriveId = 21; // Kraken X60
    public static final int FrontRightTurnId = 22; // Kraken X44 / NEO v1.2
    public static final int FrontRightEncoderId = 23; // CANCoder

    public static final int BackLeftDriveId = 11; // Kraken X60
    public static final int BackLeftTurnId = 12; // Kraken X44 / NEO v1.2
    public static final int BackLeftEncoderId = 13; // CANCoder

    public static final int BackRightDriveId = 04; // Kraken X60
    public static final int BackRightTurnId = 02; // Kraken X44 / NEO v1.2
    public static final int BackRightEncoderId = 03; // CANCoder

    public static final int PigeonGyroId = 05; // CTR Pigeon 2.0

    public static final int PivotLeaderMotorId = 41;
    public static final int PivotLeftFollowerMotorId = 42;
    public static final int PivotRight1MotorId = 43;
    public static final int PivotRight2MotorId = 44;

    public static final int ElevatorMotorId = 45;

    public static final int WristMotorId = 46;

    public static final int RollerLeaderMotorId = 47;
    public static final int RollerFollowerMotorId = 48;

    public static final int HoldSensorId = 49;

    public static final int IntakeSensorLeftId = 50;
    public static final int IntakeSensorCenterId = 51;
    public static final int IntakeSensorRightId = 52;

    public static final int PdhId = 60; // PDH
  }

  public static final class Pwm {
    // PWM Ports
    public static final int ActuatorPort = 1;

    public static final int LedUnderGlowPort = 9;
    public static final int LedTurretPort = 7;
  }

  public static final class Usb {
    // drive Input Configurations
    public static final int DrivePort = 0;
    public static final int OperatorPort = 1;
    public static final int GuitarPort = 2;
    public static final int CreditPort = 3;
  }
}
