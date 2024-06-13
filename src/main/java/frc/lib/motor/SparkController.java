package frc.lib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class SparkController extends CANSparkMax implements Motor {

    public SparkController(int deviceId, MotorType type) {
        super(deviceId, type);
    }

}
