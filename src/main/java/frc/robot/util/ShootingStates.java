package frc.robot.util;

public enum ShootingStates {
    AMP(125, 0.1, 0.5),
    SPEAKER_FRONT(80, 0.75, 0.9),
    SPEAKER_BACK(140, 0.5, 0.7),
    STAGE_FRONT(67, 0.96, 0.98);

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
/*
        opAButton.whileTrue(shooterCommands.shootNote(79.7, 0.75));
        opBButton.whileTrue(shooterCommands.shootNote(140, 0.5));
        opYButton.whileTrue(shooterCommands.shootNote(125, 0.1));
        opXButton.whileTrue(shooterCommands.shootNote(64, 1));
 */
