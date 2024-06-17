package frc.lib.generic.motor;

public class GenericTalonSRX implements Motor {
    @Override
    public void setP(double kP, int slot) {

    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {

    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {

    }

    @Override
    public void stopMotor() {

    }

    @Override
    public void setMotorPosition(double position) {

    }

    @Override
    public double getMotorPosition() {
        return 0;
    }

    @Override
    public double getMotorVelocity() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public double getSystemPosition() {
        return 0;
    }

    @Override
    public double getSystemVelocity() {
        return 0;
    }

    @Override
    public void setFollowerOf(int masterPort) {

    }

    @Override
    public void setSignalUpdateFrequency(MotorProperties.SignalType signalType, double updateFrequencyHz) {

    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        return false;
    }
}
