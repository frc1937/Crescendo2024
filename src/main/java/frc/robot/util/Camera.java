package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;

import java.util.Objects;

public final class Camera {
    private final String name;
    private final Transform3d robotToCamera;

    public Camera(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
    }

    public String name() {
        return name;
    }

    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        var that = (Camera) obj;
        return Objects.equals(this.name, that.name) &&
                Objects.equals(this.robotToCamera, that.robotToCamera);
    }

    @Override
    public int hashCode() {
        return Objects.hash(name, robotToCamera);
    }

    @Override
    public String toString() {
        return "Camera[" +
                "name=" + name + ", " +
                "robotToCamera=" + robotToCamera + ']';
    }
}
