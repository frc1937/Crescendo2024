// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Map;
import java.util.NavigableMap;

public class RobotState {
    Pose2d pose;
    ChassisSpeeds velocity;

    public RobotState(Pose2d pose, ChassisSpeeds fieldRelativeVelocity) {
        this.pose = pose;
        this.velocity = fieldRelativeVelocity;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose3d getPose3d() {
        return new Pose3d(pose);
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * Predict the pose of the robot in a bit of time
     *
     * @param futureTime How distant from now, in seconds, is the moment to predict the state
     *                     at
     */
    public static RobotState predict(TimeInterpolatableBuffer<Pose2d> poseHistory, double futureTime) {
        // Assume the robot moves and continues moving in an arc with a constant magnitude of acceleration. We believe
        // the main use case of this command is when driving in an arc near
        // the speaker.

        // The calculations are done using pose exponential integration. You
        // can read more about it in chapter 10.2 in Controls Engineering in
        // the FIRST Robotics Competition by Tyler Veness. Beware, it involves
        // advanced maths: group theory, linear algebra and differential
        // geometry to name a few.

        NavigableMap<Double, Pose2d> actualPoseSamples = poseHistory.getInternalBuffer();

        // Use three pose samples
        Map.Entry<Double, Pose2d> firstPoseSample = actualPoseSamples.firstEntry();
        Map.Entry<Double, Pose2d> lastPoseSample = actualPoseSamples.lastEntry();
        double middleTime = (firstPoseSample.getKey() + lastPoseSample.getKey()) / 2;
        Map.Entry<Double, Pose2d> middlePoseSample = actualPoseSamples.ceilingEntry(middleTime);

        // Find the twist between the first and the last ones

        // Find the twists between them
        Twist2d wholeTwist = firstPoseSample.getValue().log(lastPoseSample.getValue());
        Twist2d firstTwist = firstPoseSample.getValue().log(middlePoseSample.getValue());
        Twist2d lastTwist = middlePoseSample.getValue().log(lastPoseSample.getValue());

        // Find how grandiose were twists compared to one another
        double firstTwistSize = Math.sqrt(firstTwist.dx * firstTwist.dx + firstTwist.dy * firstTwist.dy);
        double secondTwistSize = Math.sqrt(secondTwist.dx * secondTwist.dx + secondTwist.dy * secondTwist.dy);

        if (secondTwist != 0) {
            // Extrapolate linearly to predict the size of the future twist
            double predictedTwistSize = predictLinearValue(
                futureTime,
                secondTwistSize,
                lastPoseSample.getKey(),
                firstTwist,
                middlePoseSample.getKey()
            );
            double predictedAndSecondTwistSizeRatio = predictedTwistSize / secondTwistSize;

            Twist2d predictedTwist = new Twist2d(
                predictedAndSecondTwistSizeRatio * lastTwist.dx,
                predictedAndSecondTwistSizeRatio * lastTwist.dy,
                predictedAndSecondTwistSizeRatio * lastTwist.theta
            );

            // 'Apply' the future twist onto the last pose sample
            Pose2d predictedPose = lastPoseSample.getValue().exp(predictedTwist);

            // Derive the speed prediction from the predicted twist
            double predictedTwistDuration = futureTime - lastPoseSample.getKey();
            ChassisSpeeds predictedVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    predictedTwist.dx / predictedTwistDuration,
                    predictedTwist.dy / predictedTwistDuration,
                    predictedTwist.dtheta / predictedTwistDuration,
                    predictedPose.getRotation()
            );

            return new RobotState(predictedPose, predictedVelocity);
        } else {
            // Assume the robot continues not to move
            // WARNING: In this case, the yaw is also assumed to be static, even though the robot might be rotating
            return new RobotState(lastPoseSample.getValue(), ChassisSpeeds(0, 0, 0));
        }
    }

    private static double predictLinearValue(double futureTime, double last, double lastTime, double previous, double previousTime) {
        // v(t) = mt + b
        // v(previousTime) = previous
        // v(lastTime) = last
        // m(previousTime) + b = previous
        // m(lastTime) + b = last
        // m(previousTime - lastTime) = previous - last
        double m = (previous - last) / (previousTime - lastTime);
        double b = last - m * lastTime;
        return m * futureTime + b;
    }
}