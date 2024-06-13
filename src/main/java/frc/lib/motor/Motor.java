package frc.lib.motor;

import com.revrobotics.REVLibError;

public interface Motor {
    void stopMotor();

    REVLibError restoreFactoryDefaults();
    REVLibError burnFlash();
}
