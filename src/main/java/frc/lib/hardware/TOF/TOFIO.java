package frc.lib.hardware.TOF;

// Basic distance sensor
public interface TOFIO {
    
    public void config(TOFConfig config);

    // TODO: CHANGE TO LoggedInputs PATTERN
    public boolean isDetected();

}
