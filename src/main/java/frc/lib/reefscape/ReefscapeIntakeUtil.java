// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.reefscape;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.o2026.RobotState;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class ReefscapeIntakeUtil {

  public static boolean hasNewCoralFromCollector() {
    // find all corals
    List<GamePieceOnFieldSimulation> corals = new ArrayList<>();
    for (GamePieceOnFieldSimulation coral : RobotState.getSimArena().gamePiecesOnField())
      corals.add(coral);

    // choose those close enough to intake
    for (GamePieceOnFieldSimulation coral : corals)
      if (insideIntakeRange(coral.getPose3d()))
        return RobotState.getSimArena().removeGamePiece(coral);

    return false;
  }

  public static boolean insideIntakeRange(Pose3d coralPositionInAir) {
    var pose = RobotState.getPoseEst().toPose2d();

    Translation3d robotPositionOnField = new Translation3d(pose.getTranslation());
    Rotation3d robotOrientation = new Rotation3d(pose.getRotation());
    Translation3d intakePositionOnField =
        robotPositionOnField.plus(
            new Translation3d(Inches.of(15.0), Inches.of(0.0), Inches.of(8.0))
                .rotateBy(robotOrientation));

    Translation3d difference = coralPositionInAir.getTranslation().minus(intakePositionOnField);
    return Math.abs(difference.getX()) < Units.inchesToMeters(18.0)
        && Math.abs(difference.getY()) < Units.inchesToMeters(18.0)
        && Math.abs(difference.getZ()) < Units.inchesToMeters(18.0);
  }
}
