package frc.lib.hardware.TOF;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import lombok.Getter;
import lombok.With;

@With
public class TOFConfig {

  @Getter private double minSignalStrength;
  @Getter private Distance minDistance;

  public TOFConfig() {
    minSignalStrength = 0.0;
    minDistance = Meters.of(0.0);
  }

  public TOFConfig(
    double minSignalStrength,
    Distance minDistance) {

    this.minSignalStrength = minSignalStrength;
    this.minDistance = minDistance;
  }
}