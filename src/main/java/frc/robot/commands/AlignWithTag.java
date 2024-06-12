package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PhotonCameraSource;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.swerve.Swerve5990;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;

public class AlignWithTag extends SequentialCommandGroup {
    private final Swerve5990 swerve5990;
    private final PoseEstimator5990 poseEstimator5990;
    private final PhotonCameraSource photonCameraSource;

    private static final double targetDistance = 2;

    private static final Translation3d upTranslation = new Translation3d(0.0, 0.0, 1.0);
    private static final Translation3d leftTranslation = new Translation3d(0.0, 1.0, 0.0);
    private static final Translation3d downTranslation = new Translation3d(0.0, 0.0, -1.0);
    private static final Translation3d rightTranslation = new Translation3d(0.0, -1.0, 0.0);

    public AlignWithTag(Swerve5990 swerve5990, PoseEstimator5990 poseEstimator5990, PhotonCameraSource photonCameraSource) {
        this.swerve5990 = swerve5990;
        this.poseEstimator5990 = poseEstimator5990;
        this.photonCameraSource = photonCameraSource;
    }

    public Command driveToTag(int id) {
        return new FunctionalCommand(
                () -> {
                },
                () ->
                        swerve5990.driveFieldRelative(0, 0,
                                swerve5990.getGyroAzimuth().plus(getTransformedTagPose(id))),
                interrupt -> {
                },
                () -> false,

                swerve5990
        );
    }

    public Rotation2d getTransformedTagPose(int id) {
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getBluePose();
        Optional<Pose3d> tagPose = getTagPose(robotPose, id);

        if (tagPose.isEmpty()) {
            return robotPose.getRotation();
        }

        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d differenceInXY = tagPose.get().toPose2d().getTranslation().minus(robotTranslation);

        SmartDashboard.putNumber("NIG/robotTranslation", robotTranslation.getNorm());
        SmartDashboard.putNumber("NIG/tagTranslation", tagPose.get().toPose2d().getTranslation().getNorm());
        SmartDashboard.putNumber("NIG/differenceXY", differenceInXY.getNorm());

        SmartDashboard.putNumber("NIG/atan of diff", Math.atan2(Math.abs(differenceInXY.getY()), Math.abs(differenceInXY.getX())));

        SmartDashboard.putNumber("X difference: ", differenceInXY.getX());
        SmartDashboard.putNumber("Y difference; ", differenceInXY.getY());

        Rotation2d desiredAnge = Rotation2d.fromRadians(Math.atan2(
                differenceInXY.getY(),
                -differenceInXY.getX()));

//        while (desiredAnge.getDegrees() < 0) {
//            desiredAnge.plus(Rotation2d.fromDegrees(360));
//        }

        SmartDashboard.putNumber("NIG/Desired angle (From robot): ", desiredAnge.getDegrees());
        SmartDashboard.putNumber("NIG/Current angle (Of robot):", robotPose.getRotation().getDegrees());

        return desiredAnge;

        // Determine tag rotation
//        double maxZ = 0.0;
//        int maxIndex = 0;
//        int index = 0;
//
//        for (var translation : new Translation3d[]{upTranslation, leftTranslation, downTranslation, rightTranslation}) {
//            double z = tagPose
//                            .get()
//                            .transformBy(new Transform3d(translation, new Rotation3d()))
//                            .getZ();
//
//            if (z > maxZ) {
//                maxZ = z;
//                maxIndex = index;
//            }
//
//            index++;
//        }

//        Rotation2d robotRotation = new Rotation2d(Math.PI + Math.PI / 2.0 * maxIndex);
//
//        SmartDashboard.putNumber("Get tag rotation in deg from robo: ", tagPose.get().toPose2d().getRotation().getDegrees());
//
//         Calculate robot pose
//        return new Pose2d(0, 0, tagPose.get().toPose2d().getRotation());
//        return tagPose
//                .get()
//                .toPose2d()
//                .transformBy(new Transform2d(new Translation2d(targetDistance, 0.0), robotRotation));
    }

    public Optional<Pose3d> getTagPose(Pose2d robotPose, int id) {
        PhotonTrackedTarget target = photonCameraSource.getTags();

        if (target == null || target.getFiducialId() != id) {
            return Optional.empty();
        }

        Pose3d robotPose3d = new Pose3d(robotPose);

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = robotPose3d.transformBy(ROBOT_TO_FRONT_CAMERA);

        Pose3d tagPose = cameraPose.transformBy(cameraToTarget.inverse());

        return Optional.ofNullable(tagPose);
    }
}