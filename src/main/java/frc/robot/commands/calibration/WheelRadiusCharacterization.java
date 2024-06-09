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
    private final TunableNumber characterizationSpeed = new TunableNumber("Omega Speed Wheel Characterization ", 0.6);
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

    private double lastGyroYawRadians = 0.0;
    private double accumulatedGyroYawRadians = 0.0;

    private double[] wheelStartingPositionsRadians;

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
        lastGyroYawRadians = gyroYawRadsSupplier.getAsDouble();
        accumulatedGyroYawRadians = 0.0;

        wheelStartingPositionsRadians = swerve5990.getWheelPositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        double characterizationSpeedValue = characterizationSpeed.get();
        double omegaSpeed = omegaLimiter.calculate(omegaDirection.getValue() * characterizationSpeedValue);

        swerve5990.drive(0, 0, omegaSpeed, false);

        // Get yaw and wheel positions
        accumulatedGyroYawRadians += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRadians);
        lastGyroYawRadians = gyroYawRadsSupplier.getAsDouble();

        double averageWheelPositionRadians = 0.0;
        double[] currentWheelPositions = swerve5990.getWheelPositions();

        for (int i = 0; i < 4; i++) {
            averageWheelPositionRadians += Math.abs(currentWheelPositions[i] - wheelStartingPositionsRadians[i]);
        }

        SmartDashboard.putNumber("averageWheelPosition", averageWheelPositionRadians);

        averageWheelPositionRadians /= 4.0;

        if (averageWheelPositionRadians != 0) {
            currentEffectiveWheelRadius = (accumulatedGyroYawRadians * DRIVE_BASE_RADIUS) / averageWheelPositionRadians;
        }

        SmartDashboard.putNumber("Characterization/Drive/DrivePosition", averageWheelPositionRadians);
        SmartDashboard.putNumber("Characterization/Drive/AccumGyroYawRads", accumulatedGyroYawRadians);
        SmartDashboard.putNumber("Characterization/Drive/CurrentWheelRadius [M]", currentEffectiveWheelRadius);
        SmartDashboard.putNumber("Characterization/Drive/OmegaSpeed", omegaSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        if (accumulatedGyroYawRadians <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}

