package frc.robot.util;

public enum ShootingStates {
    AMP(125, 0.08, 0.5),
    SPEAKER_FRONT(74, 0.82, 0.9),
    SPEAKER_BACK(135, 0.7, 0.7),
    STAGE_FRONT(60, 0.88, 0.98);

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
