package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FIELD_LENGTH_METRES;
import static frc.robot.util.AlliancePose2d.AllianceUtils.isBlueAlliance;

public class AlliancePose2d {
    private final Pose2d bluePose;
    private final Pose2d redPose;

    public AlliancePose2d(Pose2d pose, boolean blueAlliance) {
        if (blueAlliance) {
            bluePose = pose;
            redPose = mirrorPose(pose);
        } else {
            redPose = pose;
            bluePose = mirrorPose(pose);
        }
    }


    /**
     * Get the correct pose based on the current alliance
     */
    public Pose2d getCorrectPose() {
        return isBlueAlliance() ? bluePose : redPose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    public Pose2d getRedPose() {
        return redPose;
    }

    public static class AllianceUtils {
        private static boolean isBlueAlliance = true;
        private static double blueAllianceCheckTimestamp = -1;

        /**
         * @return whether the robot is on the blue alliance
         */
        public static boolean isBlueAlliance() {
            final double timestamp = Timer.getFPGATimestamp();

            if (timestamp - blueAllianceCheckTimestamp > 0.5) {
                isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Blue);
                blueAllianceCheckTimestamp = timestamp;
            }

            return isBlueAlliance;
        }

        /**
         * Gets the pose, assuming it is on the current alliance
         *
         * @return AlliancePose2d
         */
        public static AlliancePose2d fromCorrectPose(Pose2d pose2d) {
            return new AlliancePose2d(pose2d, isBlueAlliance());
        }

        public static AlliancePose2d fromCorrectPose(double xValue, double yValue, Rotation2d rotation) {
            return new AlliancePose2d(new Pose2d(xValue, yValue, rotation), isBlueAlliance());
        }

        public static AlliancePose2d fromBluePose(Pose2d pose) {
            return new AlliancePose2d(pose, true);
        }

        public static AlliancePose2d fromRedPose(Pose2d pose) {
            return new AlliancePose2d(pose, false);
        }

        /**
         * Mirrors a rotation across the center of the field if the current alliance is red.
         *
         * @param rotation the rotation to mirror if the current alliance is red
         * @return the rotation
         */
        public static Rotation2d getCorrectRotation(Rotation2d rotation) {
            if (isBlueAlliance())
                return rotation;
            return new Rotation2d(Math.PI).minus(rotation);
        }
    }

    private Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METRES.in(Meters) - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }
}
