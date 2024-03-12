package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_TOLERANCE;

public class RetardedPitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANCoder encoder = new CANCoder(PIVOT_CAN_CODER);
    private final PIDController controller = new PIDController(1, 0, 0);
    private double setpoint;

    public RetardedPitch() {
        SmartDashboard.putNumber("rpitch/p-value", 0);
        SmartDashboard.putNumber("rpitch/d-value", 0);


        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);

        encoder.configFactoryDefault();
        encoder.configSensorDirection(true);

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
        controller.setP(SmartDashboard.getNumber("rpitch/p-value", 0));
        controller.setD(SmartDashboard.getNumber("rpitch/d-value", 0));

        double angle = getRawPosition();

        motor.setVoltage(controller.calculate(angle, setpoint));
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getRawPosition() {
        double angle = (encoder.getAbsolutePosition() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        }

        return angle;
    }

}
