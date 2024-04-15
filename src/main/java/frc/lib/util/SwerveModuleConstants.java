package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int moduleNumber;
    public final int driveMotorID;
    public final int steerMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * SwerveSubsystem Module Constants to be used when creating swerve modules.
     * @param moduleNumber - Number of the swerve module
     * @param driveMotorID - ID of the drive motor
     * @param steerMotorID - ID of the angle motor
     * @param canCoderID - ID of the canCoder
     * @param angleOffset - Offset of canCoder to zero
     */
    public SwerveModuleConstants(int moduleNumber, int driveMotorID, int steerMotorID, int canCoderID, Rotation2d angleOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
