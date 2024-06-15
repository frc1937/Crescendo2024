package frc.lib.motor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXController extends TalonFX implements Motor {
    private int slotToUse = 0;

    private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal, currentSignal;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    public TalonFXController(int deviceId) {
        super(deviceId);

        positionSignal = super.getPosition().clone();
        velocitySignal = super.getVelocity().clone();
        voltageSignal = super.getMotorVoltage().clone();
        currentSignal = super.getSupplyCurrent().clone();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        switch (mode) { //todo: add more (NECESSARY) control types
            case PERCENTAGE_OUTPUT -> super.setControl(dutyCycleRequest.withOutput(output));
            case VOLTAGE -> super.setControl(voltageRequest.withOutput(output));

            case POSITION -> super.setControl(positionVoltageRequest.withPosition(output).withSlot(slotToUse));
            case VELOCITY -> super.setControl(velocityVoltageRequest.withVelocity(output).withSlot(slotToUse));

            case CURRENT -> throw new UnsupportedOperationException("CTRE is money hungry, and wants you to pay $150 for CURRENT control. Nuh uh!");
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        setOutput(mode, output);
        System.out.println("Phoenix v6 already does feedforward control for you !! Please use the correct method");
    }

    @Override
    public double getMotorVelocity() {
        return getSystemVelocity() / talonConfig.Feedback.SensorToMechanismRatio;
    }

    @Override
    public double getMotorPosition() {
        return getSystemPosition() / talonConfig.Feedback.SensorToMechanismRatio;
    }

    @Override
    public double getSystemPosition() {
        return positionSignal.refresh().getValue();
    }

    @Override
    public double getSystemVelocity() {
        return velocitySignal.refresh().getValue();
    }

    @Override
    public double getCurrent() {
        return currentSignal.refresh().getValue();
    }

    @Override
    public void setFollowerOf(int masterPort) {
        setControl(new StrictFollower(masterPort)); //check if this should be called 1 times or once is enough
    }

    @Override
    public void setSignalUpdateFrequency(MotorProperties.SignalType signalType, double updateFrequency) {
        switch (signalType) {
            case VELOCITY -> velocitySignal.setUpdateFrequency(updateFrequency);
            case POSITION -> positionSignal.setUpdateFrequency(updateFrequency);
            case VOLTAGE -> voltageSignal.setUpdateFrequency(updateFrequency);
            case CURRENT -> currentSignal.setUpdateFrequency(updateFrequency);
        }
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        talonConfig.MotorOutput.Inverted = configuration.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        applyCurrentLimits(configuration);

        talonConfig.Feedback.SensorToMechanismRatio = configuration.conversionFactor;
//        talonConfig.Slot0.GravityType.value = GravityTypeValue.Elevator_Static.value; This is essential for ARM/ELEVATOR control. Please look into.

        setConfig0(configuration);
        setConfig1(configuration);
        setConfig2(configuration);

        slotToUse = configuration.slotToUse;

        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = getConfigurator().apply(talonConfig);
            counter--;
        }

        optimizeBusUtilization();
        return statusCode == StatusCode.OK;
    }

    private void setConfig0(MotorConfiguration configuration) {
        talonConfig.Slot0.kP = configuration.slot0.kP();
        talonConfig.Slot0.kI = configuration.slot0.kI();
        talonConfig.Slot0.kD = configuration.slot0.kD();
        talonConfig.Slot0.kA = configuration.slot0.kA();
        talonConfig.Slot0.kS = configuration.slot0.kS();
        talonConfig.Slot0.kV = configuration.slot0.kV();
        talonConfig.Slot0.kG = configuration.slot0.kG();
    }

    private void setConfig1(MotorConfiguration configuration) {
        talonConfig.Slot1.kP = configuration.slot1.kP();
        talonConfig.Slot1.kI = configuration.slot1.kI();
        talonConfig.Slot1.kD = configuration.slot1.kD();
        talonConfig.Slot1.kA = configuration.slot1.kA();
        talonConfig.Slot1.kS = configuration.slot1.kS();
        talonConfig.Slot1.kV = configuration.slot1.kV();
        talonConfig.Slot1.kG = configuration.slot1.kG();
    }

    private void setConfig2(MotorConfiguration configuration) {
        talonConfig.Slot2.kP = configuration.slot2.kP();
        talonConfig.Slot2.kI = configuration.slot2.kI();
        talonConfig.Slot2.kD = configuration.slot2.kD();
        talonConfig.Slot2.kA = configuration.slot2.kA();
        talonConfig.Slot2.kS = configuration.slot2.kS();
        talonConfig.Slot2.kV = configuration.slot2.kV();
        talonConfig.Slot2.kG = configuration.slot2.kG();
    }

    private void applyCurrentLimits(MotorConfiguration configuration) {
        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = configuration.dutyCycleOpenLoopRampPeriod;
        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = configuration.dutyCycleCloseLoopRampPeriod;

        if (configuration.statorCurrentLimit != -1) {
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimit;
        }

        if(configuration.supplyCurrentLimit != -1) {
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimit;
        }
    }
}
