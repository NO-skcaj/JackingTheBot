// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.hardware.TOF.TOFConfig;
import frc.lib.hardware.motor.MotorConfig;

public class Configs {

  public static final class Vision {

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N4, N1> kSingleTagStdDevs =
        VecBuilder.fill(4, 4, 4, Double.MAX_VALUE);

    public static final Matrix<N4, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.4, 0.4, 1.0, Double.MAX_VALUE);
  }

  public static final class Superstructure {

    public static final TOFConfig CoralSensorConfig =
        new TOFConfig().withMinDistance(Inches.of(1.0)).withMinSignalStrength(2000.0);

    public static final double PivotGearRatio = (60.0 / 10.0) * (50.0 / 50.0) * (48.0 / 12.0);

    public static final MotorConfig PivotConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(60.0))
            .withStatorCurrent(Amps.of(160.0))
            .withBrakeMode(true)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(10.0)
            .withI(0.0)
            .withD(0.5)
            .withV(2.88)
            .withVelocityLimit(RotationsPerSecond.of(0.25))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(1))
            .withSensorToMechanismRatio(PivotGearRatio);

    public static final double ElevatorGearRatio = 3.0 * (64.0 / 18.0);

    public static final MotorConfig ElevatorConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(60.0))
            .withStatorCurrent(Amps.of(160.0))
            .withBrakeMode(true)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(10.0)
            .withI(0.0)
            .withD(0.5)
            .withV(5.12)
            .withVelocityLimit(RotationsPerSecond.of(10))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(10))
            .withSensorToMechanismRatio(ElevatorGearRatio);

    public static final double WristGearRatio = (48.0 / 8.0);

    public static final MotorConfig WristConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(60.0))
            .withStatorCurrent(Amps.of(160.0))
            .withBrakeMode(true)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(25.0)
            .withI(0.0)
            .withD(0.5)
            .withV(0.72)
            .withVelocityLimit(RotationsPerSecond.of(10))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(10))
            .withSensorToMechanismRatio(WristGearRatio);

    public static final double RollerGearRatio = (48.0 / 8.0);

    public static final MotorConfig RollerConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(40.0))
            .withStatorCurrent(Amps.of(80.0))
            .withBrakeMode(false)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(5)
            .withI(0.03)
            .withD(0.1)
            .withVelocityLimit(RotationsPerSecond.of(10))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(10))
            .withSensorToMechanismRatio(RollerGearRatio);
  }

  public static final class Chassis {

    public static final double IntakeAssistRotationPower = 0.9;
    public static final LinearVelocity IntakeAssistSpeed = FeetPerSecond.of(0.8);

    public static final Distance CenterDistToReef = Inches.of(22.5);

    public static final MotorConfig DriveConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(40.0))
            .withStatorCurrent(Amps.of(80.0))
            .withBrakeMode(false)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(0.07)
            .withV(0.0)
            .withVelocityLimit(RotationsPerSecond.of(10))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(10));

    public static final MotorConfig TurnConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(20.0))
            .withStatorCurrent(Amps.of(40.0))
            .withInverted(true)
            .withBrakeMode(true)
            .withContinuousWrap(true)
            .withP(1.0)
            .withD(0.0)
            .withSensorToMechanismRatio(21.5)
            .withVelocityLimit(RotationsPerSecond.of(150))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(200));

    public static LinearVelocity MaximumLinear = FeetPerSecond.of(12.0);

    public static LinearAcceleration MaximumLinearAcceleration = FeetPerSecondPerSecond.of(12.0);

    public static AngularVelocity MaximumAngularVelocity = RadiansPerSecond.of(4 * Math.PI);

    public static AngularAcceleration MaximumAngularAcceleration =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    static {
      if (RobotBase.isReal()) {
        MaximumLinear = MaximumLinear.div(2.0);
        MaximumLinearAcceleration = MaximumLinearAcceleration.div(2.0);
        MaximumAngularVelocity = MaximumAngularVelocity.div(2.0);
        MaximumAngularAcceleration = MaximumAngularAcceleration.div(2.0);
      }
    }

    public static final double TranslateExponentialPower = 3.0;
    public static final double AngularExponentialPower = 2.0;

    public static final PathConstraints constraints =
        new PathConstraints(
            Configs.Chassis.MaximumLinear,
            Configs.Chassis.MaximumLinearAcceleration,
            Configs.Chassis.MaximumAngularVelocity,
            Configs.Chassis.MaximumAngularAcceleration);
  }
}
