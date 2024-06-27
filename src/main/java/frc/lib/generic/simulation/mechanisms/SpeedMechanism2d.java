package frc.lib.generic.simulation.mechanisms;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;

/**
 * A Mechanism2d object to display the current velocity and target velocity of a mechanism.
 */
public class SpeedMechanism2d {
    private final String key;

    private final Mechanism2d mechanism;

    private final MechanismLigament2d
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,

    targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;

    private final double deadband;

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity) {
        this(key, maximumDisplayableVelocity, 0.001);
    }

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity, double deadband) {
        this.deadband = deadband;
        this.key = key;

        this.mechanism = new Mechanism2d(2 * maximumDisplayableVelocity, 2 * maximumDisplayableVelocity);

        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maximumDisplayableVelocity, maximumDisplayableVelocity);

        this.currentVelocityLigament = root.append(new MechanismLigament2d("CurrentVelocityLigament", 0, 0, MECHANISM_LINE_WIDTH, NO_VELOCITY_COLOUR));

        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("CurrentVelocityTopArrowLigament", SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity, TOP_WING_UPWARDS_ANGLE, MECHANISM_LINE_WIDTH, NO_VELOCITY_COLOUR));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("CurrentVelocityBottomArrowLigament", SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity, BOTTOM_WING_DOWNWARDS_ANGLE, MECHANISM_LINE_WIDTH, NO_VELOCITY_COLOUR));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, MECHANISM_LINE_WIDTH, TARGET_COLOUR));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity, TOP_WING_UPWARDS_ANGLE, MECHANISM_LINE_WIDTH, TARGET_COLOUR));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity, BOTTOM_WING_DOWNWARDS_ANGLE, MECHANISM_LINE_WIDTH, TARGET_COLOUR));
    }

    /**
     * Updates the mechanism's velocity and target velocity and logs the Mechanism2d object.
     *
     * @param velocity       the current velocity
     * @param targetVelocity the target velocity
     */
    public void updateMechanism(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        updateMechanism(velocity);
    }

    /**
     * Updates the mechanism's velocity and logs the Mechanism2d object.
     *
     * @param velocity the current velocity
     */
    public void updateMechanism(double velocity) {
        buildArrowAtLigament(velocity, currentVelocityTopArrowLigament, currentVelocityBottomArrowLigament);

        currentVelocityLigament.setLength(velocity);

        setCurrentLigamentColor(velocityToColor(velocity));

        SmartDashboard.putData("Mechanism/" + key, mechanism);
    }

    /**
     * Sets the target velocity but doesn't log the Mechanism2d object.
     *
     * @param targetVelocity the target velocity
     */
    public void setTargetVelocity(double targetVelocity) {
        buildArrowAtLigament(targetVelocity, targetVelocityTopArrowLigament, targetVelocityBottomArrowLigament);
        targetVelocityLigament.setLength(targetVelocity);
    }
    //todo: cleanup this file like what the ehll was he doing lmfao??

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return POSITIVE_VELOCITY_COLOUR;

        else if (velocity < -deadband)
            return NEGATIVE_VELOCITY_COLOUR;

        return NO_VELOCITY_COLOUR;
    }

    private void buildArrowAtLigament(double velocity, MechanismLigament2d topWing, MechanismLigament2d bottomWing) {
        if (velocity > deadband) {
            topWing.setAngle(TOP_WING_POINT_ANGLE);
            bottomWing.setAngle(BOTTOM_WING_POINT_ANGLE);
        } else if (velocity < -deadband) {
            topWing.setAngle(TOP_WING_POINT_ANGLE - 180);
            bottomWing.setAngle(BOTTOM_WING_POINT_ANGLE - 180);
        } else {
            topWing.setAngle(TOP_WING_UPWARDS_ANGLE);
            bottomWing.setAngle(BOTTOM_WING_DOWNWARDS_ANGLE);
        }
    }
}