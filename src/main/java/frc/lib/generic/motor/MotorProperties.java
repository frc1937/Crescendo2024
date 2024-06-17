package frc.lib.generic.motor;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class MotorProperties {
    public enum IdleMode {
        COAST, BRAKE
    }

    public enum SignalType {
        CURRENT, POSITION, VELOCITY, VOLTAGE, TEMPERATURE
    }

    public enum ControlMode {
        CURRENT, POSITION, VELOCITY, VOLTAGE, PERCENTAGE_OUTPUT
    }

    public enum FeedforwardType {
        SIMPLE, ARM, ELEVATOR
    }

    public enum GravityType {
        ELEVATOR, ARM
    }

    public record Slot(double kP, double kD, double kI, double kV, double kA, double kS, double kG, GravityTypeValue gravityType) { }
}
