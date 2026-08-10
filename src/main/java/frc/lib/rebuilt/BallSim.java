// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.rebuilt.firecontrol.FuelPhysicsSim;
import frc.o2026.Constants;

public class BallSim {
    
  private FuelPhysicsSim ballSim;

  private static BallSim m_inst;

  private static Supplier<ChassisSpeeds> realSimSpeeds;

  public static BallSim getInstance() {

    if (m_inst == null) m_inst = new BallSim();

    return m_inst;
  }

  // Because this is not a static function
  // "getInstance" is guarunteed to have been called
  // that means that ballSim is guarunteed to be non-null
  public void defineRobot(
    Distance robotWidth,
    Distance robotLength,
    Supplier<Pose2d> realSimPose,
    Supplier<ChassisSpeeds> realSimSpeeds,
    BooleanSupplier isIntaking,
    Runnable incIntake
  ) {

    BallSim.realSimSpeeds = realSimSpeeds;

    // tell it about your robot
    ballSim.configureRobot(
      robotWidth.in(Meters), 
      robotLength.in(Meters), 
      Inches.of(4.5).in(Meters),
      realSimPose, realSimSpeeds);

    ballSim.addIntakeZone(-0.85 / 2 - 0.2, -0.85 / 2, 
      robotWidth.in(Meters) / -2, 
      robotWidth.in(Meters) / 2, 
      isIntaking, 
      incIntake);
  }

  public FuelPhysicsSim getPhysicsSim() {
    return ballSim;
  }

  private BallSim() {

    ballSim = new FuelPhysicsSim("Fuel");

    ballSim.enable();
    ballSim.placeFieldBalls();  // spawns all the game pieces


  }

  public void update() {

    ballSim.tick();
  }

  public void launchAtRPM(Pose2d robotPose, double shooterRPM) {
    
    Translation3d launchPos = new Translation3d(robotPose.getX(), robotPose.getY(), Units.inchesToMeters(20.0));

    double launchAngleRad = Math.toRadians(90-27);
    double exitSpeed = 1.0 * shooterRPM * Math.PI * Units.inchesToMeters(5.0) / 60.0;
    double vHorizontal = exitSpeed * Math.cos(launchAngleRad);
    double vVertical = exitSpeed * Math.sin(launchAngleRad);

    double vx = vHorizontal * Math.cos(robotPose.getRotation().getRadians());
    double vy = vHorizontal * Math.sin(robotPose.getRotation().getRadians());

    Translation3d launchVel = new Translation3d(vx, vy, vVertical);

    var speeds = realSimSpeeds.get();
    launchVel = launchVel.plus(new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));

    ballSim.launchBall(launchPos, launchVel, 60.0);
  }
}
