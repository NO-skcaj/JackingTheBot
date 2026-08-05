package frc.lib.hardware.TOF;

import java.util.function.Supplier;

public class TOFIODoNothing implements TOFIO {

    private Supplier<Boolean> m_supplier;

    public TOFIODoNothing(Supplier<Boolean> supplier) {
        m_supplier = supplier;
    }

    @Override
    public void config(TOFConfig config) {
    }

    @Override
    public boolean isDetected() {

        return m_supplier.get();
    }
    
}
