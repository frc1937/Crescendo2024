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

import static frc.robot.subsystems.shooter.ShooterConstants.POSE_HISTORY_DURATION;

public class RobotStateHistory {
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(POSE_HISTORY_DURATION);
    private final NavigableMap<Double,Pose2d> samples = poseHistory.getInternalBuffer();

    public void addSample(double time, Pose2d pose) {
        poseHistory.addSample(time, pose);
    }

    public RobotState estimate() {
        Map.Entry<Double,Pose2d> lastPoseSample = samples.lastEntry();
        Map.Entry<Double,Pose2d> pastPoseSample = samples.ceilingEntry(lastPoseSample.getKey() - 0.1);
        double twistDuration = lastPoseSample.getKey() - pastPoseSample.getKey();
        Twist2d twist = pastPoseSample.getValue().log(lastPoseSample.getValue());

        return twistAndPoseToRobotState(lastPoseSample.getValue(), twist, twistDuration);
    }

    private RobotState twistAndPoseToRobotState(Pose2d pose, Twist2d twist, double twistDuration) {
        ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                twist.dx / twistDuration,
                twist.dy / twistDuration,
                twist.dtheta / twistDuration,
                pose.getRotation()
        );

        return new RobotState(pose, velocity);
    }

    /**
     * Predict the pose of the robot in a bit of time
     *
     * @param futureTime How distant from now, in seconds, is the moment to predict the state
     *                     at
     */
    public RobotState predict(double futureTime) {
        // WARNING: The yaw this code predicts is bullshit, but it doesn't matter for we don't need
        // it throughout the code.

        // Assume the robot moves and continues moving in an arc with a constant magnitude of acceleration. We believe
        // the main use case of this command is when driving in an arc near
        // the speaker.

        // The calculations are done using pose exponential integration. You
        // can read more about it in chapter 10.2 in Controls Engineering in
        // the FIRST Robotics Competition by Tyler Veness. Beware, it involves
        // advanced maths: group theory, linear algebra and differential
        // geometry to name a few.

        // Use three pose samples
        Map.Entry<Double,Pose2d> firstPoseSample = samples.firstEntry();
        Map.Entry<Double,Pose2d> lastPoseSample = samples.lastEntry();
        double middleTime = (firstPoseSample.getKey() + lastPoseSample.getKey()) / 2;
        Map.Entry<Double,Pose2d> middlePoseSample = samples.ceilingEntry(middleTime);

        // Find the twists between them
        Twist2d firstTwist = firstPoseSample.getValue().log(middlePoseSample.getValue());
        Twist2d lastTwist = middlePoseSample.getValue().log(lastPoseSample.getValue());

        // To avoid division by zero, assure lastTwist dx != 0 or dy != 0
        if (lastTwist.dx == 0 || lastTwist.dy == 0) {
            // Assume the robot continues not to move
            return new RobotState(lastPoseSample.getValue(), new ChassisSpeeds(0, 0, 0));
        }

        // Find how grandiose were twists compared to one another
        // We define the 'size' of an arc similarly to the Pythagorean theorem:
        // size = sqrt(dx^2 + dy^2)

        // Also, when the robot slows significatly, it makes more sense to predict
        // that it will slow down even more, rather than change to the opposite direction.
        // Thus, we use logarithmic extrapolation and not linear extrapolation. For example,
        // whilst linear extrapolation predicts f(3) = -1/3 given f(1) = 1 and f(2) = 1/3,
        // logarithmic extrapolation predicts f(3) = 1/9.

        // We exploit the fact that log(sqrt(x)) = log(x) / 2 to optimise computations. Also, we ignore
        // the division by 2 for now because we use linear extrapolation
        double firstTwist2LogSize = Math.log(firstTwist.dx * firstTwist.dx + firstTwist.dy * firstTwist.dy);
        double lastTwist2LogSize = Math.log(lastTwist.dx * lastTwist.dx + lastTwist.dy * lastTwist.dy);

        // Extrapolate to predict the size of the future twist
        double predictedTwist2LogSize = predictLinearValue(
            futureTime,
            lastTwist2LogSize,
            lastPoseSample.getKey(),
            firstTwist2LogSize,
            middlePoseSample.getKey()
        );

        double predictedAndLastTwistSizeRatio = Math.exp((predictedTwist2LogSize - lastTwist2LogSize) / 2);

        // Consider the last twist but with the predicted size
        Twist2d predictedTwist = new Twist2d(
            predictedAndLastTwistSizeRatio * lastTwist.dx,
            predictedAndLastTwistSizeRatio * lastTwist.dy,
            predictedAndLastTwistSizeRatio * lastTwist.dtheta
        );

        // 'Apply' the predicted twist onto the last pose sample
        Pose2d predictedPose = lastPoseSample.getValue().exp(predictedTwist);

        // Derive the speed prediction from the predicted twist
        double predictedTwistDuration = futureTime - lastPoseSample.getKey();

        return twistAndPoseToRobotState(predictedPose, predictedTwist, predictedTwistDuration);
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
