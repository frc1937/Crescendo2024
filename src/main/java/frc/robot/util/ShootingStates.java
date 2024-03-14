package frc.robot.util;

public enum ShootingStates {
    EMPTY(130, 0),
    AMP(120, 0.055),
    SPEAKER_FRONT(50, 0.4), //WORKING
    SPEAKER_BACK(137, 0.69 * 0.7), //WORK
    STAGE_FRONT(61, 0.93 * 0.98);

    private final double angle, speedPercentage;

    ShootingStates(double angle, double speedPercentage) {
        this.angle = angle;
        this.speedPercentage = speedPercentage;
    }

    public double getAngle() {
        return this.angle;
    }

    public double getSpeedPercentage() {
        return this.speedPercentage;
    }
}
