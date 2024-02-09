// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
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

        // Find the twists between them
        Twist2d firstTwist = firstPoseSample.getValue().log(middlePoseSample.getValue());
        Twist2d lastTwist = middlePoseSample.getValue().log(lastPoseSample.getValue());

        // Assume each descriptor of the twists to be a linear function of time
        // and thus guess the future twist
        Twist2d predictedTwist = new Twist2d(
                predictLinearValue(futureTime, lastTwist.dx, lastPoseSample.getKey(), firstTwist.dx, middlePoseSample.getKey()),
                predictLinearValue(futureTime, lastTwist.dy, lastPoseSample.getKey(), firstTwist.dy, middlePoseSample.getKey()),
                predictLinearValue(futureTime, lastTwist.dtheta, lastPoseSample.getKey(), firstTwist.dtheta, middlePoseSample.getKey())
        );

        // 'Apply' the future twist onto the last pose sample
        Pose2d predictedPose = lastPoseSample.getValue().exp(predictedTwist);

        // Derive the speed prediction from the predicted twist
        double nextTwistDuration = futureTime - lastPoseSample.getKey();
        ChassisSpeeds predictedVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                predictedTwist.dx / nextTwistDuration,
                predictedTwist.dy / nextTwistDuration,
                predictedTwist.dtheta / nextTwistDuration,
                predictedPose.getRotation()
        );

        return new RobotState(predictedPose, predictedVelocity);
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