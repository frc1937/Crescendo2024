package frc.lib.motor;

import com.revrobotics.*;

public class GenericSpark extends CANSparkMax implements Motor {
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private int slotToUse = 0;

    public GenericSpark(int deviceId, MotorType type) {
        super(deviceId, type);

        optimizeBusUsage();

        encoder = super.getEncoder();
        controller = super.getPIDController();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        switch (mode) { //todo: this method, correctly.
            case PERCENTAGE_OUTPUT -> controller.setReference(output, ControlType.kDutyCycle);

            case VELOCITY -> controller.setReference(output, ControlType.kVelocity, slotToUse, feedforward);
            case POSITION -> controller.setReference(output, ControlType.kPosition, slotToUse, feedforward);

            case VOLTAGE -> controller.setReference(output, ControlType.kVoltage, slotToUse);
            case CURRENT -> controller.setReference(output, ControlType.kCurrent, slotToUse);
        }
    }

    @Override
    public void setMotorPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void setP(double kP, int slot) {
        controller.setP(kP, slot);
    }

    @Override
    public double getMotorVelocity() {
        return getSystemVelocity() / encoder.getVelocityConversionFactor();
    }

    @Override
    public double getMotorPosition() {
        return getSystemPosition() / encoder.getPositionConversionFactor();
    }

    @Override
    public double getCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double getSystemPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getSystemVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setFollowerOf(int masterPort) {
        super.follow(new GenericSpark(masterPort, MotorType.kBrushless));
        super.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    }

    /**Explanation here: <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">REV DOCS</a>*/
    @Override
    public void setSignalUpdateFrequency(MotorProperties.SignalType signalType, double updateFrequency) {
        int ms = (int) (1000 / updateFrequency);

        switch (signalType) {
            case VELOCITY, CURRENT -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
            case POSITION -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, ms);
        }
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        super.restoreFactoryDefaults();

        super.setIdleMode(configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? IdleMode.kBrake : IdleMode.kCoast);
        super.setInverted(configuration.inverted);

        encoder.setPositionConversionFactor(configuration.conversionFactor);
        encoder.setVelocityConversionFactor(configuration.conversionFactor);

        super.enableVoltageCompensation(12);

        super.setSmartCurrentLimit((int) configuration.statorCurrentLimit);

        configurePID(configuration);

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configurePID(MotorConfiguration configuration) {
        controller.setP(configuration.slot0.kP(), 0);
        controller.setI(configuration.slot0.kI(), 0);
        controller.setD(configuration.slot0.kD(), 0);

        controller.setP(configuration.slot1.kP(), 1);
        controller.setI(configuration.slot1.kI(), 1);
        controller.setD(configuration.slot1.kD(), 1);

        controller.setP(configuration.slot2.kP(), 2);
        controller.setI(configuration.slot2.kI(), 2);
        controller.setD(configuration.slot2.kD(), 2);

        slotToUse = configuration.slotToUse;
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        super.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
    }
}
