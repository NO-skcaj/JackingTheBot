package frc.lib.hardware.TOF;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

public class TOFIOCANRange implements TOFIO {
    
    private CANrange m_sensor;
    private StatusSignal<Boolean> m_isDetected;

    public TOFIOCANRange(int id) {

        m_sensor = new CANrange(id);

        m_isDetected = m_sensor.getIsDetected();
    }

    public TOFIOCANRange(int id, TOFConfig config) {

        this(id);

        config(config);
    }

    @Override
    public void config(TOFConfig config) {

        m_sensor.getConfigurator().apply(
            new ProximityParamsConfigs()
                .withProximityThreshold(config.getMinDistance())
                .withMinSignalStrengthForValidMeasurement(config.getMinSignalStrength()));
    }


    // TODO: CHANGE TO LoggedInputs PATTERN
    @Override
    public boolean isDetected() {

        return m_isDetected.getValue();
    }
}
