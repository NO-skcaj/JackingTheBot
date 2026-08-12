// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.o2026.Constants;
import frc.o2026.Constants.Superstructure.ArmPosition;
import frc.o2026.RobotState;
import frc.o2026.subsystems.Superstucture.DesiredState.RollerState;
import frc.shared.Util;
import frc.shared.hardware.TOF.TOFIO;
import frc.shared.hardware.motor.MotorIO;
import frc.shared.hardware.motor.MotorIO.MotorInputs;
import frc.shared.hardware.vision.poseVision.PoseCameraIO;
import frc.shared.reefscape.ReefscapeScoring;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Superstucture extends SubsystemBase {

  /// PIVOT

  // On the left
  private final MotorIO m_pivotLeader;
  private final MotorIO m_pivotLeftFollower;

  // On the right
  private final MotorIO m_pivotRight1; // Parallel to left leader
  private final MotorIO m_pivotRight2; // Parallel to left follower

  private MotorInputs m_pivotInputs = new MotorInputs();

  /// ELEVATOR

  private MotorIO m_elevator;
  private MotorInputs m_elevatorInputs = new MotorInputs();

  /// WRIST

  private MotorIO m_wrist;
  private MotorInputs m_wristInputs = new MotorInputs();

  /// ROLLERS

  private MotorIO m_rollerLeader;
  private MotorIO m_rollerFollower;

  private MotorInputs m_rollerInputs = new MotorInputs();

  /// CORAL SENSING

  // This is the CANRange not in a row
  private TOFIO m_sensor;

  public Superstucture(
      MotorIO pivotLeader,
      MotorIO pivotLeftFollower,
      MotorIO pivotRight1,
      MotorIO pivotRight2,
      MotorIO elevator,
      MotorIO wrist,
      MotorIO rollerLeader,
      MotorIO rollerFollower,
      TOFIO sensor) {

    m_pivotLeader = pivotLeader;
    m_pivotLeftFollower = pivotLeftFollower;
    m_pivotRight1 = pivotRight1;
    m_pivotRight2 = pivotRight2;

    m_elevator = elevator;

    m_wrist = wrist;

    m_rollerLeader = rollerLeader;
    m_rollerFollower = rollerFollower;

    m_sensor = sensor;

    m_pivotLeftFollower.follow(m_pivotLeader.getId(), false);
    m_pivotRight1.follow(m_pivotLeader.getId(), true);
    m_pivotRight2.follow(m_pivotLeader.getId(), true);

    m_rollerFollower.follow(m_rollerLeader.getId(), true);

    resetAtStarting();
  }

  public static enum DesiredState {
    startPos,
    home,
    groundIntake,
    L1F,
    L2F,
    L3F,
    L4F,
    L2B,
    L3B,
    L4B;

    public static enum RollerState {
      intakingCoral,
      scoringCoral,
      holdingCoral,
      intakingAlgae,
      scoringAlgae,
      holdingAlgae,
      idle
    }

    @Getter private RollerState roller;

    public DesiredState with(RollerState newRoller) {
      roller = newRoller;
      return this;
    }
  }

  public static enum MeasuredState {
    start,
    transitioning,
    atSetPoint;
  }

  private DesiredState m_desiredState = DesiredState.startPos.with(RollerState.idle);
  private MeasuredState m_measuredState = MeasuredState.start;

  // private DesiredState m_lastDesiredState = m_desiredState;

  public void setState(DesiredState newState) {

    if (newState != null && newState.getRoller() != null) {
      m_desiredState = newState;
      Logger.recordOutput("Superstructure/d-state", m_desiredState.toString());
      Logger.recordOutput("Superstructure/d-roller", m_desiredState.roller.toString());
    }
    ;
  }

  public DesiredState getState() {
    return m_desiredState;
  }

  public MeasuredState getMeasuredState() {

    return m_measuredState;
  }

  @Override
  public void periodic() {

    m_pivotLeader.periodic();
    m_pivotLeftFollower.periodic();
    m_pivotRight1.periodic();
    m_pivotRight2.periodic();

    m_elevator.periodic();

    m_wrist.periodic();

    m_rollerLeader.periodic();
    m_rollerFollower.periodic();

    m_pivotLeader.updateInputs(m_pivotInputs);
    Logger.processInputs("Superstructure/pivot", m_pivotInputs);

    m_elevator.updateInputs(m_elevatorInputs);
    Logger.processInputs("Superstructure/elevator", m_elevatorInputs);

    m_wrist.updateInputs(m_wristInputs);
    Logger.processInputs("Superstructure/wrist", m_wristInputs);

    m_rollerLeader.updateInputs(m_rollerInputs);
    Logger.processInputs("Superstructure/roller", m_rollerInputs);

    Logger.recordOutput("Superstructure/isAtSetpoint", isAtSetpoint());

    if (!isAtSetpoint()) m_measuredState = MeasuredState.transitioning;
    else m_measuredState = MeasuredState.atSetPoint;
    Logger.recordOutput("Superstructure/m-state", m_measuredState.toString());
    Logger.recordOutput("Superstructure/m-roller", m_desiredState.roller.toString());

    var pos = getPosFromState();

    Logger.recordOutput("Superstructure/d-pivot", pos.pivot().in(Rotations));
    Logger.recordOutput(
        "Superstructure/d-elevator",
        pos.elevator().in(Meters) * Constants.Superstructure.ElevatorMetersToRotations);
    Logger.recordOutput("Superstructure/d-wrist", pos.wrist().in(Rotations));

    var volts =
        switch (m_desiredState.getRoller()) {
          case intakingCoral -> Constants.Superstructure.IntakingCoralVolt;
          case scoringCoral -> Constants.Superstructure.ScoringCoralVolt;
          case holdingCoral -> Constants.Superstructure.HoldingCoralVolt;
          case intakingAlgae -> Constants.Superstructure.IntakingAlgaeVolt;
          case scoringAlgae -> Constants.Superstructure.ScoringAlgaeVolt;
          case holdingAlgae -> Constants.Superstructure.HoldingAlgaeVolt;
          default -> Volts.of(0.0);
        };

    RobotState.setSimIntaking(
        m_desiredState == DesiredState.groundIntake && m_measuredState == MeasuredState.atSetPoint);
    Logger.recordOutput("SimIntaking", RobotState.isSimIntaking());
    Logger.recordOutput("HasCoral", RobotState.isHasCoral());

    Logger.recordOutput("Superstructure/d-roller", volts.toString());

    m_pivotLeader.setPosition(pos.pivot());
    m_elevator.setPosition(
        Rotations.of(
            pos.elevator().in(Meters) * Constants.Superstructure.ElevatorMetersToRotations));
    m_wrist.setPosition(pos.wrist());

    m_rollerLeader.setVoltage(volts);

    visualize();
    ReefscapeScoring.getInstance().setHeldCoral(getCoral());
  }

  public ArmPosition getPosFromState() {

    return switch (m_desiredState) {
      case startPos -> Constants.Superstructure.Starting;
      case home -> Constants.Superstructure.Home;
      case groundIntake -> Constants.Superstructure.GroundIntake;
      case L1F -> Constants.Superstructure.L1F;
      case L2F -> Constants.Superstructure.L2F;
      case L3F -> Constants.Superstructure.L3F;
      case L4F -> Constants.Superstructure.L4F;
      case L2B -> Constants.Superstructure.L2B;
      case L3B -> Constants.Superstructure.L3B;
      case L4B -> Constants.Superstructure.L4B;
      default -> Constants.Superstructure.Home;
    };
  }

  public int armLevelFromState() {
    return switch (m_desiredState) {
      case L1F -> 1;
      case L2F -> 2;
      case L3F -> 3;
      case L4F -> 4;
      case L2B -> 2;
      case L3B -> 3;
      case L4B -> 4;
      default -> 0;
    };
  }

  public boolean isAtSetpoint() {

    double pivotErrorDeg = getPosFromState().pivot().minus(m_pivotInputs.position).abs(Degrees);

    double elevatorActualMeters =
        m_elevatorInputs.position.in(Rotations)
            / Constants.Superstructure.ElevatorMetersToRotations;
    double elevatorErrorMeters =
        Math.abs(getPosFromState().elevator().in(Meters) - elevatorActualMeters);

    double wristErrorDeg = getPosFromState().wrist().minus(m_wristInputs.position).abs(Degrees);

    // Check if they're under absolute tolerance
    return pivotErrorDeg < 3.0 && elevatorErrorMeters < 0.03 && wristErrorDeg < 3.0;
  }

  public boolean isBackwardsToScore() {

    var pose = RobotState.getPoseEst().toPose2d();

    var hub =
        Util.flipOnRed(new Pose2d(Constants.Field.BlueHub, Rotation2d.kZero)).getTranslation();

    var angle = pose.getRotation().minus(pose.getTranslation().minus(hub).getAngle());

    return Math.abs(angle.getDegrees()) < 90;
  }

  public Rotation2d getEffectorRotation() {

    return isBackwardsToScore() ? Rotation2d.kZero : Rotation2d.k180deg;
  }

  public Optional<Pose2d> getDrivePointToScore(boolean isRightSide) {

    List<Pose2d> poses =
        List.of(IntStream.range(6, 12), IntStream.range(17, 23)).stream()
            .map(IntStream::boxed)
            .flatMap(
                (reef) -> {
                  return reef.map(
                      (tagID) -> {
                        Pose2d tagPose = PoseCameraIO.getTagPose(tagID).toPose2d();

                        Distance yOffset = Constants.Superstructure.CoralScoreYOffset;
                        if (isRightSide) {
                          yOffset = yOffset.times(-1);
                        }
                        Translation2d offsetFromTag =
                            new Translation2d(Constants.Superstructure.CoralScoreXOffset, yOffset);

                        var transformedPose =
                            tagPose.plus(
                                new Transform2d(
                                    offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

                        return new Pose2d(
                            transformedPose.getTranslation(),
                            transformedPose.getRotation().plus(getEffectorRotation()));
                      });
                })
            .toList();

    if (poses.isEmpty()) return Optional.empty();
    else return Optional.of(RobotState.getPoseEst().toPose2d().nearest(poses));
  }

  public void resetAtStarting() {

    m_pivotLeader.resetEncoder(Constants.Superstructure.Starting.pivot());
    m_elevator.resetEncoder(
        Rotations.of(
            Constants.Superstructure.Starting.elevator().in(Meters)
                * Constants.Superstructure.ElevatorMetersToRotations));
    m_wrist.resetEncoder(Constants.Superstructure.Starting.wrist());
  }

  public Command resetAtStartingCmd() {

    return runOnce(this::resetAtStarting);
  }

  public Command coralIntake() {

    return runOnce(() -> setState(DesiredState.groundIntake.with(RollerState.intakingCoral)))
        .andThen(new WaitUntilCommand(m_sensor::isDetected))
        .andThen(() -> setState(DesiredState.groundIntake.with(RollerState.holdingCoral)));
  }

  private static Time AnimationTime = Seconds.of(0.5);

  public Command score(boolean isRight) {

    return new WaitUntilCommand(this::isAtSetpoint)
        .andThen(
            defer(
                () -> {
                  Pose3d fieldStartPos = getCoral().orElse(RobotState.getPoseEst());
                  String pipeId =
                      String.valueOf(RobotState.nearestReefTagFiducial())
                          + String.valueOf(armLevelFromState())
                          + (isRight ? "R" : "L");
                  Logger.recordOutput("lastPipViz", pipeId);
                  Pose3d fieldEndPos =
                      ReefscapeScoring.getInstance().getReefs().get(pipeId).getFirst();

                  Command scoreAction =
                      run(() -> setState(m_desiredState.with(RollerState.scoringCoral)))
                          .until(() -> !m_sensor.isDetected())
                          .withTimeout(1.0)
                          .andThen(runOnce(() -> setState(m_desiredState.with(RollerState.idle))));

                  Timer vizAnimator = new Timer();

                  Command vizStart =
                      Util.runOnce(
                          () -> {
                            vizAnimator.start();
                          });

                  Command vizRun =
                      Util.run(
                          () ->
                              ReefscapeScoring.getInstance()
                                  .setHeldCoral(
                                      Optional.of(
                                          new Pose3d(
                                              fieldStartPos
                                                  .getTranslation()
                                                  .interpolate(
                                                      fieldEndPos.getTranslation(),
                                                      vizAnimator.get()
                                                          / AnimationTime.in(Seconds)),
                                              fieldStartPos
                                                  .getRotation()
                                                  .interpolate(
                                                      fieldEndPos.getRotation(),
                                                      vizAnimator.get()
                                                          / AnimationTime.in(Seconds))))));

                  var updateSim = Util.runOnce(() -> {
                    ReefscapeScoring.getInstance().score(pipeId);
                    RobotState.setHasCoral(false);
                  });

                  return vizStart
                      .andThen(
                          scoreAction.alongWith(
                              vizRun
                                  .withDeadline(new WaitCommand(AnimationTime))
                                  .andThen(updateSim)));
                }));
  }

  public Command home() {

    return runOnce(() -> setState(DesiredState.home.with(RollerState.idle)));
  }

  public Command L1() {

    return runOnce(() -> setState(DesiredState.L1F.with(RollerState.idle)));
  }

  public Command L2() {

    return defer(
        () -> {
          if (isBackwardsToScore())
            return runOnce(() -> setState(DesiredState.L2F.with(RollerState.idle)));
          else return runOnce(() -> setState(DesiredState.L2B.with(RollerState.idle)));
        });
  }

  public Command L3() {

    return defer(
        () -> {
          if (isBackwardsToScore())
            return runOnce(() -> setState(DesiredState.L3F.with(RollerState.idle)));
          else return runOnce(() -> setState(DesiredState.L3B.with(RollerState.idle)));
        });
  }

  public Command L4() {

    return defer(
        () -> {
          if (isBackwardsToScore())
            return runOnce(() -> setState(DesiredState.L4F.with(RollerState.idle)));
          else return runOnce(() -> setState(DesiredState.L4B.with(RollerState.idle)));
        });
  }

  public void visualize() {

    Pose3d pivotPos =
        new Pose3d(
            Inches.of(1.25),
            Inches.of(0),
            Inches.of(-1.0),
            new Rotation3d(Degrees.of(0), Degrees.of(4.914357), Degrees.of(180.0)));

    pivotPos =
        pivotPos.rotateAround(
            new Translation3d(Inches.of(11.0), Inches.of(0.0), Inches.of(13.0)),
            new Rotation3d(
                Degrees.of(0), m_pivotInputs.position.minus(Degrees.of(90)), Degrees.of(0)));

    var elevatorDistance =
        Meters.of(
            m_elevatorInputs.position.in(Rotations)
                / Constants.Superstructure.ElevatorMetersToRotations);

    Pose3d elevator1Pos =
        new Pose3d(
            Inches.of(1.25),
            Inches.of(0),
            elevatorDistance.div(2.0).plus(Inches.of(-18.3)),
            new Rotation3d(Degrees.of(0), Degrees.of(4.914357), Degrees.of(180.0)));

    elevator1Pos =
        elevator1Pos.rotateAround(
            new Translation3d(Inches.of(11.0), Inches.of(0.0), Inches.of(13.05)),
            new Rotation3d(
                Degrees.of(0), m_pivotInputs.position.minus(Degrees.of(90)), Degrees.of(0)));

    Pose3d elevator2Pos =
        new Pose3d(
            Inches.of(1.25),
            Inches.of(0),
            elevatorDistance.plus(Inches.of(-35.5)),
            new Rotation3d(Degrees.of(0), Degrees.of(4.914357), Degrees.of(180.0)));

    elevator2Pos =
        elevator2Pos.rotateAround(
            new Translation3d(Inches.of(11.0), Inches.of(0.0), Inches.of(13.05)),
            new Rotation3d(
                Degrees.of(0), m_pivotInputs.position.minus(Degrees.of(90)), Degrees.of(0)));

    Pose3d wristPos =
        new Pose3d(
            new Translation3d(Inches.of(11.15), Inches.of(0.0), Inches.of(13.0)), Rotation3d.kZero);

    wristPos =
        wristPos.plus(
            new Transform3d(
                new Translation3d(
                        elevatorDistance.plus(Inches.of(28.0)), Inches.of(0.0), Inches.of(5.0))
                    .rotateBy(
                        new Rotation3d(
                            Degrees.of(0),
                            m_pivotInputs.position.plus(Degrees.of(180)),
                            Degrees.of(0))),
                new Rotation3d(
                    Degrees.of(180),
                    m_wristInputs.position.times(-1).plus(m_pivotInputs.position),
                    Degrees.of(0))));

    Logger.recordOutput("viz/pivot", pivotPos);
    Logger.recordOutput("viz/elevator1", elevator1Pos);
    Logger.recordOutput("viz/elevator2", elevator2Pos);
    Logger.recordOutput("viz/wrist", wristPos);
  }

  public Optional<Pose3d> getCoral() {
    if (!m_sensor.isDetected()) return Optional.empty();

    var elevatorDistance =
        Meters.of(
            m_elevatorInputs.position.in(Rotations)
                / Constants.Superstructure.ElevatorMetersToRotations);

    // Start with the robot pos
    Pose3d wristPos = RobotState.getPoseEst();

    // Go to the pivot
    wristPos =
        wristPos.plus(
            new Transform3d(
                new Translation3d(Inches.of(11.15), Inches.of(0.0), Inches.of(13.0)),
                Rotation3d.kZero));

    // Go to the wrist from the pivot base
    wristPos =
        wristPos.plus(
            new Transform3d(
                new Translation3d(
                        elevatorDistance.plus(Inches.of(28.0)), Inches.of(0.0), Inches.of(5.0))
                    .rotateBy(
                        new Rotation3d(
                            Degrees.of(0),
                            m_pivotInputs.position.plus(Degrees.of(180)),
                            Degrees.of(0))),
                new Rotation3d(
                    Degrees.of(180),
                    m_wristInputs.position.times(-1).plus(m_pivotInputs.position),
                    Degrees.of(0))));

    return Optional.of(wristPos);
  }
}
