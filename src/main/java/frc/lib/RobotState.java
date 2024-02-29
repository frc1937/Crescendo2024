// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.NavigableMap;

public class RobotState {
    Pose2d pose;
    ChassisSpeeds velocity;

    public RobotState(Pose2d pose, ChassisSpeeds fieldRelativeVelocity) {
        this.pose = pose;
        this.velocity = fieldRelativeVelocity;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
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

        NavigableMap<Double, Pose2d> actualPoseSamples = poseHistory.getInternalBuffer();

        // Use three pose samples
        Map.Entry<Double, Pose2d> firstPoseSample = actualPoseSamples.firstEntry();
        Map.Entry<Double, Pose2d> lastPoseSample = actualPoseSamples.lastEntry();

        SmartDashboard.putNumber("last delta pos", translation(firstPoseSample, lastPoseSample));

        double middleTime = (firstPoseSample.getKey() + lastPoseSample.getKey()) / 2;
        Map.Entry<Double, Pose2d> middlePoseSample = actualPoseSamples.ceilingEntry(middleTime);

        // Find the twists between them
        Twist2d firstTwist = firstPoseSample.getValue().log(middlePoseSample.getValue());
        Twist2d lastTwist = middlePoseSample.getValue().log(lastPoseSample.getValue());

        SmartDashboard.putNumber("firstTwist thingy", Math.sqrt(firstTwist.dx * firstTwist.dx + firstTwist.dy * firstTwist.dy));
        SmartDashboard.putNumber("lastTwist thingy", Math.sqrt(lastTwist.dx * lastTwist.dx + lastTwist.dy * lastTwist.dy));
        SmartDashboard.putNumber("first translation", translation(firstPoseSample, middlePoseSample));
        SmartDashboard.putNumber("last translation", translation(middlePoseSample, lastPoseSample));

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
        double predictedTwistSize = Math.exp(predictedTwist2LogSize / 2);
        double predictedAndSecondTwistSizeRatio = predictedTwistSize / lastTwist2LogSize;

        // Consider the last twist but with the predicted size
        Twist2d predictedTwist = new Twist2d(
            predictedAndSecondTwistSizeRatio * lastTwist.dx,
            predictedAndSecondTwistSizeRatio * lastTwist.dy,
            predictedAndSecondTwistSizeRatio * lastTwist.dtheta
        );

        // 'Apply' the predicted twist onto the last pose sample
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
    }

    private static double translation(Map.Entry<Double, Pose2d> firstPoseSample, Map.Entry<Double, Pose2d> lastPoseSample) {
        return firstPoseSample.getValue().minus(lastPoseSample.getValue()).getTranslation().getNorm();
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