package frc.robot;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class StateSpacePivotController {
    private final LinearSystemLoop<N1, N1, N1> loop;
    private double setpointRadiansPerSecond = 0;

    public StateSpacePivotController(DCMotor gearbox, double momentOfInertia, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        loop = createLoop(gearbox, momentOfInertia, gearRatio, modelAccuracy, encoderAccuracy, maximumErrorTolerance, maximumControlEffort);
    }

    //todo: Adjust this to account for PIVOT differences. 

    public void reset(double measuredVelocityRotationsPerSecond) {
        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRotationsPerSecond);
        loop.reset(VecBuilder.fill(measuredVelocityRadiansPerSecond));
    }

    public double calculate(double measuredVelocityRotationsPerSecond, double setpointRotationsPerSecond) {
        setSetpoint(setpointRotationsPerSecond);
        return calculate(measuredVelocityRotationsPerSecond);
    }

    public double calculate(double measuredVelocityRotationsPerSecond) {
        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRotationsPerSecond);

        loop.setNextR(VecBuilder.fill(setpointRadiansPerSecond));
        loop.correct(VecBuilder.fill(measuredVelocityRadiansPerSecond));
        loop.predict(0.02);

        return loop.getU(0);
    }

    public void setSetpoint(double setpointRotationsPerSecond) {
        setpointRadiansPerSecond = Units.rotationsToRadians(setpointRotationsPerSecond);
    }

    private LinearSystemLoop<N1, N1, N1> createLoop(DCMotor gearbox, double momentOfInertia, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(gearbox, momentOfInertia, gearRatio);
        final LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(maximumErrorTolerance), VecBuilder.fill(maximumControlEffort), 0.02);
        final KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(modelAccuracy), VecBuilder.fill(encoderAccuracy), 0.02);

        return new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, 0.02);
    }
}