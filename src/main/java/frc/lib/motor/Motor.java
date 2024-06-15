package frc.lib.motor;

public interface Motor {
    void setP(double kP, int slot);

    void setOutput(MotorProperties.ControlMode controlMode, double output);
    void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward);
    void stopMotor();

    void setMotorPosition(double position);

    /** No gearing applied*/
    double getMotorPosition();
    /** No gearing applied*/
    double getMotorVelocity();

    double getCurrent();

    /** Gearing applied*/
    double getSystemPosition();
    /** Gearing applied*/
    double getSystemVelocity();

    void setFollowerOf(int masterPort);
    void setSignalUpdateFrequency(MotorProperties.SignalType signalType, double updateFrequency);

    boolean configure(MotorConfiguration configuration);
}
