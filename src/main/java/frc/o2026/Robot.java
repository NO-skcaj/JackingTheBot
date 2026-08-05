// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.hardware.SimBattery;
import frc.lib.hardware.motor.ctre.OrchestraOrchestrator;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    m_robotContainer = new RobotContainer();
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    OrchestraOrchestrator.sendChooser();
  }

  @Override
  public void robotInit() {

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    SimBattery.calculateSupplyVoltage();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAuto();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {

    SimulatedArena.getInstance().simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    OrchestraOrchestrator.sendChooser();
  }
}
