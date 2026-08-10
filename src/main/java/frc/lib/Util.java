// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FlippingUtil;

public class Util {

  public static void sendLambda(String name, Runnable runnable) {
    SmartDashboard.putData(lambdaAsCommand(runnable));
  }

  public static Command lambdaAsCommand(Runnable runnable) {
    return Commands.run(runnable).asProxy();
  }
  
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
        == DriverStation.Alliance.Red;
  }

  public static Pose2d flipOnRed(Pose2d pose) {
    return isRed() ? FlippingUtil.flipFieldPose(pose) : pose;
  }
}
