package frc.lib.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;

public class TalonFXController extends TalonFX implements Motor {
    public TalonFXController(int deviceId) {
        super(deviceId);
    }

    @Override
    public REVLibError restoreFactoryDefaults() {
        throw new UnsupportedOperationException("Talons cant be rev'd");
    }

    @Override
    public REVLibError burnFlash() {
        throw new UnsupportedOperationException("Talons cant be rev'd");
    }
}
