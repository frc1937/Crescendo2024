package frc.robot.commands.calibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_BASE_RADIUS;

public class WheelRadiusCharacterization extends Command {
    private static final double characterizationSpeed = 0.1; // ("WheelRadiusCharacterization/SpeedRadsPerSec",
    private static final double driveRadius = DRIVE_BASE_RADIUS;

    private static DoubleSupplier gyroYawRadsSupplier;

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value = 0;

        Direction(int value) {
        }
    }

    private final Swerve5990 swerve5990;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

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

        startWheelPositions = swerve5990.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0,
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeed)
        );

        swerve5990.driveSelfRelative(chassisSpeeds);

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();

        double averageWheelPosition = 0.0;
        double[] wheelPositions = swerve5990.getWheelRadiusCharacterizationPosition();

        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }

        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;

        SmartDashboard.putNumber("Characterization/Drive/DrivePosition", averageWheelPosition);
        SmartDashboard.putNumber("Characterization/Drive/AccumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber("Characterization/Drive/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
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

