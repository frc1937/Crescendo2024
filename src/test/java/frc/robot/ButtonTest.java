package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class ButtonTest {
    private  double shooterLength = 2;

    @Test
    void testButton() {
        DriverStation.silenceJoystickConnectionWarning(true);
//
//        double rpm = Conversions.RPMFromTangentialVelocity(MetersPerSecond.of(2), Inches.of(4));
//
//        System.out.println(rpm);
//
//        Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();
//
//        for (AprilTag aprilTag : AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags())
//            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
//
//
////        for (AprilTagFieldLayout.TagPosition tagPosition : AprilTagFieldLayout()) {
//        System.out.println("Y:" + TAG_ID_TO_POSE.get(7).getY()); //Y:5.547867999999999
//        System.out.println("X:" + TAG_ID_TO_POSE.get(7).getX()); //X:-0.038099999999999995
////        }
//
//        //Testing noteExitPos

        int[] testValuesYaw = {0, 90, 180, 270};
        int[] testValuesPitch = {0, 20, 40, 90};

        for (int i = 0; i < 4; i++) {
            Pose2d robotPose = new Pose2d(100, 5, Rotation2d.fromDegrees(testValuesYaw[i]));
            for (int j = 0; j < 4; j++) {
                double targetPitchAngle = testValuesPitch[j];

                System.out.println("Initial parameters:");
                System.out.println("Robot (x, y, azimuth): " + robotPose.getX() + ", " + robotPose.getY() + ", " + robotPose.getRotation().getDegrees());
                System.out.println("Pitch angle: " + targetPitchAngle + "\n");

                Pose3d noteExitPos = calculateShooterEndEffectorFieldRelativePose(robotPose, targetPitchAngle);
                Pose3d noteExitPose2 = getNoteExitPoseRobodox599(robotPose, targetPitchAngle);

                printPose3dNicely(noteExitPos, "verifiedExitPos");
                System.out.println();
                printPose3dNicely(noteExitPose2, "elysiumExitPose");

                Assertions.assertEquals(noteExitPos, noteExitPose2);
            }
        }

        AprilTag.generate36h11AprilTagImage(3);
    }

    private void printPose3dNicely(Pose3d pose, String name) {
        System.out.println("Pose3d " + name + ":");
        System.out.println("(" + pose.getX() + ", " + pose.getY() + ", " + pose.getZ() + ")"
                + ", ROLL" + toDegrees(pose.getRotation().getX())
                + ", PITCH" + toDegrees(pose.getRotation().getY())
                + ", YAW " + toDegrees(pose.getRotation().getZ()));
    }

    private double toDegrees(double radians) {
        return Math.round(radians * 180 / Math.PI);
    }
//    private Pose3d getNoteExitPoseTEST(Pose2d robotPose, double targetAngle) {
//        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);
//        Pose3d robotPose3d = new Pose3d(robotPose);
//
//        Transform3d robotToPivot = new Transform3d( //todo: Tune from cad
//                -0.2,
//                0,
//                0.3,
//                new Rotation3d(0, 0, 0)
//        );
//
//        Pose3d pivotPose = robotPose3d.transformBy(robotToPivot);
//
//        double pitchLength = 0.6; //todo: Tune from cad
//
//        Transform3d pivotToShooterEnd = new Transform3d(
//                pitchLength,
//                0,
//                0,
//                new Rotation3d(0, -pitchAngle.getRadians(), 0)
//        );
//
//        Pose3d noteExitPos = pivotPose.plus(pivotToShooterEnd);
//
//        return noteExitPos;
//    }

    private Pose3d getNoteExitPoseRobodox599(Pose2d robotPose, double targetAngle) {
        Rotation2d azimuthAngleToSpeaker = robotPose.getRotation();
        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);

        Pose3d robotPose3d = new Pose3d(new Pose2d(robotPose.getTranslation(), azimuthAngleToSpeaker));

        double PIVOT_POINT_Z_OFFSET = 0.5;
        double PIVOT_POINT_X_OFFSET = -1;

        Transform3d robotToPivot = new Transform3d(
                PIVOT_POINT_X_OFFSET, 0, PIVOT_POINT_Z_OFFSET,
                new Rotation3d(0, -pitchAngle.getRadians(), 0)
        );

        Transform3d pivotToEndEffector = new Transform3d(shooterLength, 0, 0, new Rotation3d());

        Pose3d shooterEndPose = new Pose3d().transformBy(robotToPivot).plus(pivotToEndEffector);
        Transform3d robotToShooterEnd = shooterEndPose.minus(new Pose3d());

        return robotPose3d.transformBy(robotToShooterEnd);
    }

    private Pose3d calculateShooterEndEffectorFieldRelativePose(Pose2d robotPose, double targetAngle) {
        Rotation2d azimuthAngleToSpeaker = robotPose.getRotation();
        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);

        Pose3d ROBOT_RELATIVE_PIVOT_POINT = new Pose3d(-1, 0, 0.5, new Rotation3d(0, 0, 0));

        Pose3d pivotPoint = ROBOT_RELATIVE_PIVOT_POINT
                .transformBy(
                        new Transform3d(new Translation3d(),
                                new Rotation3d(0, -pitchAngle.getRadians(), 0)));

        Transform3d pivotToEndEffector = new Transform3d(shooterLength, 0, 0, new Rotation3d());

        Pose3d endEffectorSelfRelativePose = pivotPoint.plus(pivotToEndEffector);
        Transform3d robotToEndEffector = endEffectorSelfRelativePose.minus(new Pose3d());
        Pose3d predictedPose = new Pose3d(new Pose2d(robotPose.getTranslation(), azimuthAngleToSpeaker));

        return predictedPose.transformBy(robotToEndEffector);
    }
}
