// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuitarController {

  private CommandXboxController m_controller;

  public GuitarController(int port) {

    m_controller = new CommandXboxController(port);
  }

  public Trigger A() {

    return m_controller.b();
  }

  public Trigger D() {

    return m_controller.x();
  }

  public Trigger G() {

    return m_controller.y();
  }

  public Trigger B() {

    return m_controller.leftBumper();
  }

  public Trigger E() {

    return m_controller.rightBumper().or(m_controller.a());
  }
}
