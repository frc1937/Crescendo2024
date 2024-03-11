//package frc.trigon.robot.utilities;
//
//import edu.wpi.first.math.Nat;
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.controller.LinearQuadraticRegulator;
//import edu.wpi.first.math.estimator.KalmanFilter;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.numbers.N2;
//import edu.wpi.first.math.system.LinearSystem;
//import edu.wpi.first.math.system.LinearSystemLoop;
//import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.system.plant.LinearSystemId;
//import edu.wpi.first.math.util.Units;
//
//public class StateSpacePivotController {
//    private final double PERIODIC_TIME_SECONDS = 0.02;
//    private final LinearSystemLoop<N2, N1, N1> loop;
//    private double setpointRadiansPerSecond = 0;
//
//    public StateSpacePivotController(DCMotor gearbox, double JKgSquaredMeters, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
//        loop = createLoop(gearbox, JKgSquaredMeters, gearRatio, modelAccuracy, encoderAccuracy, maximumErrorTolerance, maximumControlEffort);
//    }
//
//    public void reset(double measuredVelocityRotationsPerSecond) {
//        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRotationsPerSecond);
//        loop.reset(VecBuilder.fill(measuredVelocityRadiansPerSecond));
//    }
//
//    public double calculate(double measuredVelocityRotationsPerSecond, double setpointRotationsPerSecond) {
//        setSetpoint(setpointRotationsPerSecond);
//        return calculate(measuredVelocityRotationsPerSecond);
//    }
//
//    public double calculate(double measuredVelocityRotationsPerSecond) {
//        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRotationsPerSecond);
//
//        loop.setNextR(VecBuilder.fill(setpointRadiansPerSecond));
//        loop.correct(VecBuilder.fill(measuredVelocityRadiansPerSecond));
//        loop.predict(PERIODIC_TIME_SECONDS);
//
//        return loop.getU(0);
//    }
//
//    public void setSetpoint(double setpointRotationsPerSecond) {
//        setpointRadiansPerSecond = Units.rotationsToRadians(setpointRotationsPerSecond);
//    }
//
//    private LinearSystemLoop<N2, N1, N1> createLoop(DCMotor gearbox, double JKgSquaredMeters, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
//        final LinearSystem<N2, N1, N1> pitchPlant = LinearSystemId.createSingleJointedArmSystem(gearbox, JKgSquaredMeters, gearRatio);
//        final LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(pitchPlant, VecBuilder.fill(maximumErrorTolerance), VecBuilder.fill(maximumControlEffort), PERIODIC_TIME_SECONDS);
//        final KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), pitchPlant, VecBuilder.fill(modelAccuracy), VecBuilder.fill(encoderAccuracy), PERIODIC_TIME_SECONDS);
//
//        return new LinearSystemLoop<>(pitchPlant, controller, observer, 12.0, PERIODIC_TIME_SECONDS);
//    }
//}