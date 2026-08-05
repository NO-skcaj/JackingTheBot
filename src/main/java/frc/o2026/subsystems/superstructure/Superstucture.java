package frc.o2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.Alliance;
import frc.lib.hardware.TOF.TOFIO;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.hardware.motor.MotorIO.MotorInputs;
import frc.lib.hardware.vision.poseVision.PoseCameraIO;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import frc.o2026.Constants.Superstructure.ArmPosition;
import frc.o2026.subsystems.superstructure.Superstucture.DesiredState.RollerState;
import lombok.Getter;

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
        MotorIO pivotLeader, MotorIO pivotLeftFollower, 
        MotorIO pivotRight1, MotorIO pivotRight2,
        MotorIO elevator,
        MotorIO wrist,
        MotorIO rollerLeader, MotorIO rollerFollower,
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

    private DesiredState m_desiredState = DesiredState.startPos;
    private MeasuredState m_measuredState = MeasuredState.start;

    private DesiredState m_lastDesiredState = null;

    public void setState(DesiredState newState) {

        if (newState != null && newState.getRoller() != null)
            m_desiredState = newState;
    }

    public DesiredState getState() { return m_desiredState; }

    public MeasuredState getMeasuredState() { 

        return m_measuredState; 
    }

    @Override
    public void periodic() {

        m_pivotLeader.updateInputs(m_pivotInputs);
        Logger.processInputs("Superstructure/pivot", m_pivotInputs);
        
        m_elevator.updateInputs(m_elevatorInputs);
        Logger.processInputs("Superstructure/elevator", m_elevatorInputs);
        
        m_wrist.updateInputs(m_wristInputs);
        Logger.processInputs("Superstructure/wrist", m_wristInputs);
        
        m_rollerLeader.updateInputs(m_rollerInputs);
        Logger.processInputs("Superstructure/wrist", m_rollerInputs);

        if (!isAtSetpoint())
            m_measuredState = MeasuredState.transitioning;
        else if (m_desiredState == DesiredState.startPos && isAtSetpoint())
            m_measuredState = MeasuredState.start;
        else if (isAtSetpoint())
            m_measuredState = MeasuredState.atSetPoint;
        // No else needed because that is logically impossible

        // Only control motors if requests are new
        if (!m_lastDesiredState.equals(m_desiredState) &&
            !m_lastDesiredState.getRoller().equals(m_desiredState.getRoller())) {

            var pos = getPosFromState();

            var volts = switch (m_desiredState.getRoller()) {
                case intakingCoral -> Constants.Superstructure.IntakingCoralVolt;
                case scoringCoral -> Constants.Superstructure.ScoringCoralVolt;
                case holdingCoral -> Constants.Superstructure.HoldingCoralVolt;
                case intakingAlgae -> Constants.Superstructure.IntakingAlgaeVolt;
                case scoringAlgae -> Constants.Superstructure.ScoringAlgaeVolt;
                case holdingAlgae -> Constants.Superstructure.HoldingAlgaeVolt;
                default -> Volts.of(0.0);
            };
            
            m_pivotLeader.setPosition(pos.pivot());
            m_elevator.setPosition(Rotations.of(pos.elevator().in(Meters) * Constants.Superstructure.ElevatorMetersToRotations));
            m_wrist.setPosition(pos.pivot());

            m_rollerLeader.setVoltage(volts);
            
            // Remember last state
            m_lastDesiredState = m_desiredState;
        }
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

    public boolean isAtSetpoint() {

        // Take the percent error
        // 100 * ((Desired - Actual) / Actual)

        double pivotPercentError = 
            100.0 * 
                (getPosFromState().pivot().minus(m_pivotInputs.position).abs(Degrees) 
                    / m_pivotInputs.position.in(Degrees));
        
        double elevatorPercentError = 
            100.0 * 
                (Rotations.of(getPosFromState().elevator().in(Meters) / Constants.Superstructure.ElevatorMetersToRotations).minus(m_elevatorInputs.position).abs(Degrees) 
                    / m_elevatorInputs.position.in(Degrees));

        double wristPercentError = 
            100.0 * 
                (getPosFromState().wrist().minus(m_wristInputs.position).abs(Degrees) 
                    / m_wristInputs.position.in(Degrees));
        
        // Check if they're under tolerance
        return pivotPercentError < 5.0 &&
            elevatorPercentError < 5.0 &&
            wristPercentError < 5.0;
    }

    public Rotation2d getEffectorRotation() {

        if (m_desiredState == DesiredState.L1F &&
            m_desiredState == DesiredState.L2F &&
            m_desiredState == DesiredState.L3F &&
            m_desiredState == DesiredState.L4F &&
            m_desiredState == DesiredState.groundIntake) {

            return Rotation2d.kZero;

        } else if (m_desiredState == DesiredState.L2B &&
                   m_desiredState == DesiredState.L3B &&
                   m_desiredState == DesiredState.L4B) {
            
            return Rotation2d.k180deg;
        } else {
            
            return Rotation2d.kZero;
        }
    }

    public boolean isBackwardsToScore() {

        var translation = RobotState.getPoseEst().toPose2d().getTranslation();

        var hub = Alliance.flipOnRed(new Pose2d(Constants.Field.BlueHub, Rotation2d.kZero)).getTranslation();

        var angle = translation.minus(hub).getAngle();

        return Math.abs(angle.getRadians()) > Math.PI;
    }

    public Optional<Pose2d> getDrivePointToScore(boolean isRightSide) {

        List<Pose2d> poses = RobotState.getLastSeenTags()
            .stream()
            .map((tagID) -> {

                if (tagID >= 1 && tagID <= 22) {
                    Pose2d tagPose = PoseCameraIO.getTagPose(tagID).toPose2d();

                    Distance yOffset = Constants.Superstructure.CoralScoreYOffset;
                    if (isRightSide) {
                        yOffset = yOffset.times(-1);
                    }
                    Translation2d offsetFromTag = new Translation2d(Constants.Superstructure.CoralScoreXOffset, yOffset);

                    var transformedPose =
                            tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

                    return new Pose2d(
                            transformedPose.getTranslation(),
                            transformedPose.getRotation().plus(isBackwardsToScore() ? Rotation2d.k180deg : Rotation2d.kZero));
                } else {
                    return Pose2d.kZero;
                }
            })
            .filter((pose) -> ! pose.equals(Pose2d.kZero))
            .toList();
        
        if (poses.isEmpty())
            return Optional.empty();
        else
            return Optional.of(RobotState.getPoseEst().toPose2d().nearest(poses));
    }

    public void resetAtStarting() {

        m_pivotLeader.resetEncoder(Constants.Superstructure.Starting.pivot());
        m_elevator.resetEncoder(Rotations.of(Constants.Superstructure.Starting.elevator().in(Meters) * Constants.Superstructure.ElevatorMetersToRotations));
        m_wrist.resetEncoder(Constants.Superstructure.Starting.pivot());
    }

    public Command resetAtStartingCmd() {

        return runOnce(this::resetAtStarting);
    }

    public Command coralIntake() {

        return runOnce(() -> setState(DesiredState.groundIntake.with(RollerState.intakingCoral)))
        .andThen(new WaitUntilCommand(m_sensor::isDetected))
        .andThen(() -> setState(DesiredState.groundIntake.with(RollerState.holdingCoral)));
    }
    
    public Command score() {

        return defer(() -> {
            if (m_desiredState != DesiredState.groundIntake &&
                m_desiredState != DesiredState.groundIntake &&
                m_desiredState != DesiredState.groundIntake &&
                m_sensor.isDetected()) {

                return run(() -> setState(m_desiredState.with(RollerState.scoringCoral)))
                    .until(() -> !m_sensor.isDetected())
                    .withTimeout(1.0)
                    .andThen(() -> setState(m_desiredState.with(RollerState.idle)));
            } else {
                return Commands.none();
            }
        });
    }

    public Command home() {

        return runOnce(() -> setState(DesiredState.home.with(RollerState.idle)));
    }

    public Command L1() {

        return runOnce(() -> setState(DesiredState.L1F.with(RollerState.idle)));
    }

    public Command L2() {

        return defer(() -> {
            if (isBackwardsToScore())
                return runOnce(() -> setState(DesiredState.L2F.with(RollerState.idle)));
            else
                return runOnce(() -> setState(DesiredState.L2B.with(RollerState.idle)));
        });
    }

    public Command L3() {

        return defer(() -> {
            if (isBackwardsToScore())
                return runOnce(() -> setState(DesiredState.L3F.with(RollerState.idle)));
            else
                return runOnce(() -> setState(DesiredState.L3B.with(RollerState.idle)));
        });
    }

    public Command L4() {

        return defer(() -> {
            if (isBackwardsToScore())
                return runOnce(() -> setState(DesiredState.L4F.with(RollerState.idle)));
            else
                return runOnce(() -> setState(DesiredState.L4B.with(RollerState.idle)));
        });
    }
}
