package frc.lib.motor;

public class MotorProperties {
    public enum IdleMode {
        COAST, BRAKE
    }

    public enum SignalType {
        CURRENT, POSITION, VELOCITY, VOLTAGE

    }

    public enum ControlMode {
        CURRENT, POSITION, VELOCITY, VOLTAGE, PERCENTAGE_OUTPUT
    }

    public enum FeedforwardType {
        SIMPLE, ARM, ELEVATOR
    }

    public record Slot(double kP, double kD, double kI, double kV, double kA, double kS, double kG) {}
}
