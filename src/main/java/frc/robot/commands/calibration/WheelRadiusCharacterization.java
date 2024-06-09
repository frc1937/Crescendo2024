package frc.robot.commands.calibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_BASE_RADIUS;

public class WheelRadiusCharacterization extends Command {
    private final TunableNumber characterizationSpeed = new TunableNumber("Characterization/Drive/Characterization Speed ", 0.1);
    private final DoubleSupplier gyroYawRadsSupplier;

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;

        Direction(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private final Swerve5990 swerve5990;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] wheelStartingPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(Swerve5990 swerve5990, Direction omegaDirection) {
        this.swerve5990 = swerve5990;
        this.omegaDirection = omegaDirection;

        gyroYawRadsSupplier = () -> swerve5990.getGyroAzimuth().getRadians();

        addRequirements(swerve5990);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        wheelStartingPositions = swerve5990.getWheelPositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        double characterizationSpeedValue = characterizationSpeed.get();
        double omegaSpeed = omegaLimiter.calculate(omegaDirection.getValue() * characterizationSpeedValue);

        swerve5990.drive(0, 0, omegaSpeed, false);

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();

        double averageWheelPosition = 0.0;
        double[] currentWheelPositions = swerve5990.getWheelPositions();

        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(currentWheelPositions[i] - wheelStartingPositions[i]);
        }

        averageWheelPosition /= 4.0;

        if(averageWheelPosition != 0) {
            currentEffectiveWheelRadius = (accumGyroYawRads * DRIVE_BASE_RADIUS) / averageWheelPosition;
        }

        SmartDashboard.putNumber("Characterization/Drive/DrivePosition", averageWheelPosition);
        SmartDashboard.putNumber("Characterization/Drive/AccumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber("Characterization/Drive/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
        SmartDashboard.putNumber("Characterization/Drive/OmegaSpeed", omegaSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}

