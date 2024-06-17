package frc.lib.generic.motor;

import frc.lib.generic.Properties;

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

    /** Get the current running through the motor (SUPPLY current)*/
    double getCurrent();

    /** Get the temperature of the motor, in Celsius */
    double getTemperature();

    /** Gearing applied*/
    double getSystemPosition();
    /** Gearing applied*/
    double getSystemVelocity();

    void setFollowerOf(int masterPort);
    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);

    boolean configure(MotorConfiguration configuration);
}
