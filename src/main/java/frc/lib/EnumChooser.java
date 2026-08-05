// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;

public class EnumChooser<E extends Enum<E>> {

  private SendableChooser<E> m_chooser;

  public EnumChooser(String name, E defaultOption) {

    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption(defaultOption.toString(), defaultOption);

    for (E value : defaultOption.getDeclaringClass().getEnumConstants()) {
      m_chooser.addOption(value.toString(), value);
    }

    SmartDashboard.putData(name, m_chooser);
  }

  public E getSelected() {

    return m_chooser.getSelected();
  }

  public void onChange(Consumer<E> listener) {

    m_chooser.onChange(listener);
  }
}
