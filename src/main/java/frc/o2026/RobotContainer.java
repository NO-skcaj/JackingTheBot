// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.GuitarController;
import frc.lib.Util;
import frc.lib.NONBenevolentSalesman;
import frc.lib.hardware.TOF.TOFIOCANRange;
import frc.lib.hardware.gyro.GyroIOPigeon;
import frc.lib.hardware.vision.objectVision.ObjectCameraIOPhoton;
import frc.lib.hardware.vision.objectVision.ObjectCameraIOSim;
import frc.lib.hardware.vision.poseVision.PoseCameraIOLimelight;
import frc.lib.hardware.vision.poseVision.PoseCameraIOPhoton;
import frc.lib.hardware.vision.poseVision.PoseCameraIOSim;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.Swerve.DesiredState;
import frc.o2026.subsystems.superstructure.Superstucture;

public class RobotContainer extends SubsystemBase {

  private Swerve m_swerve;
  private Superstucture m_superstructure;

  private static enum Robot {
    Real,
    DevBot,
    Sim
  }

  private static Robot m_impl = Robot.DevBot;

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  //   private CommandXboxController m_operator = new
  // CommandXboxController(Constants.Usb.OperatorPort);
  private GuitarController m_guitar = new GuitarController(Constants.Usb.GuitarPort);
  private NONBenevolentSalesman m_creditOrDebit =
      new NONBenevolentSalesman(Constants.Usb.CreditPort);

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
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                    new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOPhoton(Constants.Vision.WebCam),
                    new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig));

        m_superstructure = new Superstucture(
          null, null, null, null,
           null, 
           null, 
           null, null, 
           new TOFIOCANRange(0, Configs.Superstructure.CoralSensorConfig));

        break;

      case DevBot:
        m_swerve =
            new Swerve(
                RobotBase.isReal()
                    ? new SwerveIOReal(
                        new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                        new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                        new PoseCameraIOPhoton(Constants.Vision.WebCam),
                        new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair))
                    : new SwerveIOSim(
                        new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                        new PoseCameraIOSim(Constants.Vision.WebCam),
                        new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)),
                RobotBase.isReal()
                    ? new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig)
                    : new ObjectCameraIOSim(Constants.Vision.BackCamConfig));
        break;

      case Sim:
        m_swerve =
            new Swerve(
                new SwerveIOSim(
                    new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOSim(Constants.Vision.WebCam),
                    new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOSim(Constants.Vision.BackCamConfig));
        break;
    }

    // AUTOS

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // DEFAULT COMMANDS

    m_swerve.setDefaultCommand(m_swerve.runOnce(() -> m_swerve.setState( Swerve.DesiredState.driveDefault.with(getSpeeds()))));

    // CONTROLLER BINDINGS

    m_driver.rightStick().onTrue(m_swerve.resetGyro());
    m_driver.leftStick().onTrue(m_swerve.toggleFieldCentricity());



    Util.sendLambda("yUp",    () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(-0.5, 0.0, 0.0))));
    Util.sendLambda("yDown",  () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.5, 0.0, 0.0))));
    Util.sendLambda("xLeft",  () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, -0.5, 0.0))));
    Util.sendLambda("xRight", () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.5, 0.0))));

    Util.sendLambda("rotLeft",  () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.0, Math.PI / 2))));
    Util.sendLambda("rotRight", () -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.0, -Math.PI / 2))));

    m_guitar.A().onTrue(Util.lambdaAsCommand(() -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(-0.5, 0.0, 0.0)))));
    m_guitar.D().onTrue(Util.lambdaAsCommand(() -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.5, 0.0, 0.0)))));
    m_guitar.G().onTrue(Util.lambdaAsCommand(() -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, -0.5, 0.0)))));
    m_guitar.B().onTrue(Util.lambdaAsCommand(() -> m_swerve.setState(
      DesiredState.driveRobot.with(new ChassisSpeeds(0.0, 0.5, 0.0)))));

    m_creditOrDebit
        .swipe()
        .onTrue(
            m_swerve.defer(
                () -> {
                  return Commands.race(
                      Util.lambdaAsCommand(() -> m_swerve.setState(Swerve.DesiredState.aim.with(m_swerve.getHeading().plus(Rotation2d.k180deg)))),
                      new WaitCommand(10));
                }));

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
