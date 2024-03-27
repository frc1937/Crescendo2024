package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;

public record Camera(String name, Transform3d robotToCamera) {
}
