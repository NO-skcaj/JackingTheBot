// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.reefscape;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.Triple;
import frc.lib.hardware.vision.poseVision.PoseCameraIO;
import frc.o2026.Constants;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import lombok.Getter;

public class ReefscapeScoring {

  @Getter private HashMap<Pose3d, Boolean> scoringLocations;

  private static ReefscapeScoring m_inst = null;

  private Optional<Pose3d> m_heldCoral = Optional.empty();

  public static ReefscapeScoring getInstance() {

    if (m_inst == null) {
      m_inst = new ReefscapeScoring();
    }

    return m_inst;
  }

  private ReefscapeScoring() {

    scoringLocations = getReefs();
  }

  public HashMap<Pose3d, Boolean> getReefs() {

    HashMap<Pose3d, Boolean> map = new HashMap<>();
    List.of(IntStream.range(6, 12), IntStream.range(17, 23)).stream()
        .map(IntStream::boxed)
        .forEach(
            (reef) -> {
              reef.map(PoseCameraIO::getTagPose)
                  .map(Pose3d::toPose2d)
                  .flatMap(
                      (tagPose) -> {
                        return List.of(
                                Constants.Superstructure.CoralScoreYOffset,
                                Constants.Superstructure.CoralScoreYOffset.times(-1))
                            .stream()
                            .flatMap(
                                yOffset -> {
                                  return List.of(
                                          // Height, Depth, Pitch
                                          new Triple<Distance, Distance, Angle>(
                                              Centimeters.of(71),
                                              Centimeters.of(-41),
                                              Degrees.of(35)),
                                          new Triple<Distance, Distance, Angle>(
                                              Centimeters.of(111),
                                              Centimeters.of(-35),
                                              Degrees.of(35)),
                                          new Triple<Distance, Distance, Angle>(
                                              Centimeters.of(173),
                                              Centimeters.of(-27),
                                              Degrees.of(90)),
                                          new Triple<Distance, Distance, Angle>(
                                              Centimeters.of(50),
                                              Centimeters.of(-35),
                                              Degrees.of(-30)))
                                      .stream()
                                      .map(
                                          heightAndDepthAndPitch -> {
                                            var transformedPose =
                                                tagPose.plus(
                                                    new Transform2d(
                                                        heightAndDepthAndPitch
                                                            .getSecond()
                                                            .plus(
                                                                Constants.Field.CoralDiameter.times(
                                                                    2.0)),
                                                        yOffset,
                                                        Rotation2d.kZero));

                                            return new Pose3d(
                                                new Translation3d(
                                                    transformedPose.getMeasureX(),
                                                    transformedPose.getMeasureY(),
                                                    heightAndDepthAndPitch.getFirst()),
                                                new Rotation3d(
                                                    Degrees.of(0),
                                                    heightAndDepthAndPitch.getThird().unaryMinus(),
                                                    tagPose.getRotation().getMeasure()));
                                          });
                                });
                      })
                  .forEach(loc -> map.put(loc, false));
            });
    return map;
  }

  public List<Pose3d> getCoral() {

    var scatteredCoral =
        scoringLocations.entrySet().stream()
            .filter(loc -> loc.getValue())
            .map(
                (loc) -> {
                  return loc.getKey();
                })
            .toList();

    if (m_heldCoral.isPresent()) {
      scatteredCoral =
          Stream.concat(scatteredCoral.stream(), List.of(m_heldCoral.get()).stream()).toList();
    }

    return scatteredCoral;
  }

  public void setHeldCoral(Optional<Pose3d> heldCoral) {

    m_heldCoral = heldCoral;
  }
}
