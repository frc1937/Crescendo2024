package frc.robot.util;

public enum ShootingStates {
    EMPTY(30, 0.7, 0.7),
    AMP(120, 0.055, 1),
    SPEAKER_FRONT(76, 0.72, 0.9), //WORKING
    SPEAKER_BACK(137, 0.69, 0.7), //WORK
    STAGE_FRONT(61, 0.93, 0.98);

    private final double angle, speedPercentage, rpmProportion;

    ShootingStates(double angle, double speedPercentage, double rpmProportion) {
        this.angle = angle;
        this.speedPercentage = speedPercentage;
        this.rpmProportion = rpmProportion;
    }

    public double getAngle() {
        return this.angle;
    }

    public double getSpeedPercentage() {
        return this.speedPercentage;
    }

    public double getRpmProportion() {
        return this.rpmProportion;
    }
}
