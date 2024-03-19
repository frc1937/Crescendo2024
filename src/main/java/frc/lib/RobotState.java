// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
}