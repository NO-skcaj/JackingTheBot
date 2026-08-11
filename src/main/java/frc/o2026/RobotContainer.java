// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.shared.hardware.TOF.TOFIOCANRange;
import frc.shared.hardware.TOF.TOFIOSim;
import frc.shared.hardware.gyro.GyroIOPigeon;
import frc.shared.hardware.motor.MotorIONothing;
import frc.shared.hardware.motor.ctre.MotorIOElevatorSim;
import frc.shared.hardware.motor.ctre.MotorIOSim;
import frc.shared.hardware.motor.ctre.MotorIOTalonFX;
import frc.shared.hardware.motor.rev.MotorIOSparkMax;
import frc.shared.hardware.vision.objectVision.ObjectCameraIOPhoton;
import frc.shared.hardware.vision.objectVision.ObjectCameraIOSim;
import frc.shared.hardware.vision.objectVision.ObjectVision;
import frc.shared.hardware.vision.poseVision.PoseCameraIOLimelight;
import frc.shared.hardware.vision.poseVision.PoseCameraIOPhoton;
import frc.shared.hardware.vision.poseVision.PoseCameraIOSim;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.Swerve.DesiredState;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.SwerveModule;
import frc.o2026.subsystems.superstructure.Superstucture;

public class RobotContainer extends SubsystemBase {

  private Swerve m_swerve;
  private Superstucture m_superstructure;

  private static enum Robot {
    Real,
    DevBot,
    Sim
  }

  private static Robot m_impl = Robot.Sim;

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  //   private CommandXboxController m_operator = new
  // CommandXboxController(Constants.Usb.OperatorPort);
  //   private GuitarController m_guitar = new GuitarController(Constants.Usb.GuitarPort);
  //   private NONBenevolentSalesman m_creditOrDebit =
  //       new NONBenevolentSalesman(Constants.Usb.CreditPort);

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0);

  enum Limiting {
    TrigExp,
    Linear
  }

  Limiting m_limit = Limiting.TrigExp;

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    switch (m_impl) {
      case Real:
        m_swerve =
            new Swerve(
                new SwerveIOReal(
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.FrontRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(Constants.CanIds.FrontRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontRightEncoderId,
                        Constants.Chassis.FrontRightForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.FrontLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(Constants.CanIds.FrontLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontLeftEncoderId,
                        Constants.Chassis.FrontLeftForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.BackRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(Constants.CanIds.BackRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackRightEncoderId,
                        Constants.Chassis.BackRightForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.BackLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(Constants.CanIds.BackLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackLeftEncoderId,
                        Constants.Chassis.BackLeftForwardsAngle),
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                    new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOPhoton(Constants.Vision.WebCam),
                    new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig, Constants.Field.CoralDiameter));

        m_superstructure =
            new Superstucture(
                // PIVOT
                new MotorIOTalonFX(
                    Constants.CanIds.PivotLeaderMotorId, Configs.Superstructure.PivotConfig),
                new MotorIOTalonFX(
                    Constants.CanIds.PivotLeftFollowerMotorId, Configs.Superstructure.PivotConfig),
                new MotorIOTalonFX(
                    Constants.CanIds.PivotRight1MotorId, Configs.Superstructure.PivotConfig),
                new MotorIOTalonFX(
                    Constants.CanIds.PivotRight2MotorId, Configs.Superstructure.PivotConfig),
                // ELEVATOR
                new MotorIOTalonFX(
                    Constants.CanIds.ElevatorMotorId, Configs.Superstructure.ElevatorConfig),
                // WRIST
                new MotorIOTalonFX(
                    Constants.CanIds.WristMotorId, Configs.Superstructure.WristConfig),
                // ROLLERS
                new MotorIOTalonFX(
                    Constants.CanIds.RollerLeaderMotorId, Configs.Superstructure.RollerConfig),
                new MotorIOTalonFX(
                    Constants.CanIds.RollerFollowerMotorId, Configs.Superstructure.RollerConfig),
                // SENSOR
                new TOFIOCANRange(
                    Constants.CanIds.HoldSensorId, Configs.Superstructure.CoralSensorConfig));

        break;

      case DevBot:
        m_swerve =
            new Swerve(
                new SwerveIOReal(
                    new SwerveModule(
                    new MotorIOTalonFX(Constants.CanIds.FrontRightDriveId, Configs.Chassis.DriveConfig),
                    new MotorIOSparkMax(Constants.CanIds.FrontRightTurnId, Configs.Chassis.TurnConfig),
                    Constants.CanIds.FrontRightEncoderId,
                    Constants.Chassis.FrontRightForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.FrontLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(Constants.CanIds.FrontLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontLeftEncoderId,
                        Constants.Chassis.FrontLeftForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.BackRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(Constants.CanIds.BackRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackRightEncoderId,
                        Constants.Chassis.BackRightForwardsAngle),
                    new SwerveModule(
                        new MotorIOTalonFX(Constants.CanIds.BackLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(Constants.CanIds.BackLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackLeftEncoderId,
                        Constants.Chassis.BackLeftForwardsAngle),
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                    new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOPhoton(Constants.Vision.WebCam),
                    new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig, Constants.Field.CoralDiameter)
            );

        m_superstructure =
            new Superstucture(
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),
                new TOFIOSim(RobotState::isHasCoral));
        break;

      case Sim:
        m_swerve =
            new Swerve(
                new SwerveIOSim(
                    new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOSim(Constants.Vision.WebCam),
                    new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOSim(Constants.Vision.BackCamConfig, RobotState.getSimArena()));

        m_superstructure =
            new Superstucture(
                // PIVOT
                new MotorIOSim(
                    Constants.CanIds.PivotLeaderMotorId,
                    Configs.Superstructure.PivotConfig,
                    true,
                    0.1,
                    4,
                    Configs.Superstructure.PivotGearRatio),
                new MotorIONothing(),
                new MotorIONothing(),
                new MotorIONothing(),

                // ELEVATOR
                new MotorIOElevatorSim(
                    Constants.CanIds.ElevatorMotorId,
                    Configs.Superstructure.ElevatorConfig,
                    true,
                    0.06,
                    1,
                    Configs.Superstructure.ElevatorGearRatio,
                    Inches.of(0.0),
                    Inches.of(40.0),
                    Pounds.of(20.0),
                    Meters.of(Constants.Superstructure.ElevatorSpoolRadiusMeters)),

                // WRIST
                new MotorIOSim(
                    Constants.CanIds.WristMotorId,
                    Configs.Superstructure.WristConfig,
                    false,
                    0.01,
                    1,
                    Configs.Superstructure.WristGearRatio),

                // ROLLERS
                new MotorIOSim(
                    Constants.CanIds.RollerLeaderMotorId,
                    Configs.Superstructure.RollerConfig,
                    false,
                    0.007,
                    1,
                    Configs.Superstructure.RollerGearRatio),
                new MotorIONothing(),
                // SENSOR
                new TOFIOSim(RobotState::isHasCoral));

        break;
    }

    // AUTOS

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // DEFAULT COMMANDS

    m_swerve.setDefaultCommand(
        m_swerve
            .run(() -> m_swerve.setState(Swerve.DesiredState.driveDefault.with(getSpeeds())))
            .repeatedly());

    // CONTROLLER BINDINGS

    m_driver.rightStick().onTrue(m_swerve.resetGyro().asProxy());
    m_driver.leftStick().onTrue(m_swerve.toggleFieldCentricity().asProxy());

    m_driver.y().onTrue(m_superstructure.L4());
    m_driver.x().onTrue(m_superstructure.L3());
    m_driver.b().onTrue(m_superstructure.L2());
    m_driver.a().onTrue(m_superstructure.L1());

    m_driver
        .leftTrigger()
        .and(m_swerve::hasObjects)
        .whileTrue(
            m_swerve
                .run(
                    () -> m_swerve.setState(Swerve.DesiredState.intakeAssist.with(getSpeeds()))));

    m_driver
        .leftTrigger()
        .whileTrue(m_superstructure.coralIntake())
        .onFalse(m_superstructure.home());

    Function<Boolean, Command> autoScore = (isRight) ->
        m_swerve.defer(() -> {
                  var pointOpt = m_superstructure.getDrivePointToScore(isRight);
                  if (pointOpt.isPresent())
                    return m_swerve
                        .run(() -> m_swerve.setState(DesiredState.pidPose.with(pointOpt.get())))
                        .repeatedly()
                        .until(m_swerve::isAtPidPose)
                        .andThen(
                            m_superstructure
                                .score()
                                .alongWith(
                                    m_swerve.run(
                                        () ->
                                            m_swerve.setState(
                                                DesiredState.pidPose.with(pointOpt.get())))));
                  else
                    return idle().asProxy();
                });

    m_driver
        .rightBumper()
        .whileTrue(autoScore.apply(true));

    m_driver
        .leftBumper()
        .whileTrue(autoScore.apply(false));

    // Util.sendLambda(
    //     "yUp",
    //     () -> m_swerve.setState(DesiredState.driveRobot.with(new ChassisSpeeds(-0.5, 0.0,
    // 0.0))));
    // Util.sendLambda(
    //     "yDown",
    //     () -> m_swerve.setState(DesiredState.driveRobot.with(new ChassisSpeeds(0.5, 0.0, 0.0))));
    // Util.sendLambda(
    //     "xLeft",
    //     () -> m_swerve.setState(DesiredState.driveRobot.with(new ChassisSpeeds(0.0, -0.5,
    // 0.0))));
    // Util.sendLambda(
    //     "xRight",
    //     () -> m_swerve.setState(DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.5, 0.0))));

    // Util.sendLambda(
    //     "rotLeft",
    //     () ->
    //         m_swerve.setState(
    //             DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.0, Math.PI / 2))));
    // Util.sendLambda(
    //     "rotRight",
    //     () ->
    //         m_swerve.setState(
    //             DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.0, -Math.PI / 2))));

    // m_guitar.A()
    //     .onTrue(
    //         Util.lambdaAsCommand(
    //             () ->
    //                 m_swerve.setState(
    //                     DesiredState.driveRobot.with(new ChassisSpeeds(-0.5, 0.0, 0.0)))));
    // m_guitar.D()
    //     .onTrue(
    //         Util.lambdaAsCommand(
    //             () ->
    //                 m_swerve.setState(
    //                     DesiredState.driveRobot.with(new ChassisSpeeds(0.5, 0.0, 0.0)))));
    // m_guitar.G()
    //     .onTrue(
    //         Util.lambdaAsCommand(
    //             () ->
    //                 m_swerve.setState(
    //                     DesiredState.driveRobot.with(new ChassisSpeeds(0.0, -0.5, 0.0)))));
    // m_guitar.B()
    //     .onTrue(
    //         Util.lambdaAsCommand(
    //             () ->d
    //                 m_swerve.setState(
    //                     DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.5, 0.0)))));

    // m_creditOrDebit
    //     .swipe()
    //     .onTrue(
    //         m_swerve.defer(
    //             () -> {
    //               return Commands.race(
    //                   Util.lambdaAsCommand(
    //                       () ->
    //                           m_swerve.setState(
    //                               Swerve.DesiredState.aim.with(
    //                                   m_swerve.getHeading().plus(Rotation2d.k180deg)))),
    //                   new WaitCommand(10));
    //             }));

    // PATHING COMMANDS & TRIGGERS

  }

  private ChassisSpeeds getSpeeds() {

    double leftY = MathUtil.applyDeadband(m_driver.getLeftY(), 0.01);
    double leftX = MathUtil.applyDeadband(m_driver.getLeftX(), 0.01);
    double rightX = MathUtil.applyDeadband(m_driver.getRightX(), 0.01);

    double strafe, forwards, rot;

    switch (m_limit) {
      case TrigExp:
        double angle = Math.atan2(leftY, leftX);
        double magnitude = Math.sqrt(Math.pow(leftY, 2) + Math.pow(leftX, 2));

        magnitude =
            Math.pow(Math.abs(magnitude), Configs.Chassis.TranslateExponentialPower) * magnitude;

        strafe = magnitude * Math.sin(angle);
        forwards = magnitude * Math.cos(angle);

        rot =
            -Math.pow(Math.abs(m_driver.getRightX()), Configs.Chassis.AngularExponentialPower)
                * m_driver.getRightX();
        break;
      default:
        forwards = m_xLimiter.calculate(leftX);
        strafe = m_yLimiter.calculate(leftY);
        rot = m_rotLimiter.calculate(rightX);
        break;
    }

    return new ChassisSpeeds(
        Configs.Chassis.MaximumLinear.times(strafe),
        Configs.Chassis.MaximumLinear.times(forwards),
        Configs.Chassis.MaximumAngularVelocity.times(rot));
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
