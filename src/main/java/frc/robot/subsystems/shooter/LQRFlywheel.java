package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.TIME_DIFFERENCE;
//TODO: Idk if this will even work. Worth testing tho!
public class LQRFlywheel {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> flywheelPlant;

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> observer;
    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> controller;
    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> loop;

    private Measure<Velocity<Angle>> goal;

    public LQRFlywheel(int id, double kS, double kV, double kA) {
        flywheelPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
        observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                flywheelPlant,
                VecBuilder.fill(3.0), // How accurate we think our model is
                VecBuilder.fill(0.01), // How accurate we think our encoder data is
                TIME_DIFFERENCE);

        controller = new LinearQuadraticRegulator<>(
                        flywheelPlant,
                        VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
                        // this to more heavily penalize state excursion, or make the controller behave more
                        // aggressively.
                        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
                        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                        // starting point because that is the (approximate) maximum voltage of a battery.
                        TIME_DIFFERENCE); // Nominal time between loops. 0.020 for TimedRobot, but can be lower

        loop = new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, TIME_DIFFERENCE);

        motor = new CANSparkFlex(id, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);

    }

    public void periodic() {
        //in rad per sec
        loop.setNextR(VecBuilder.fill(goal.in(RadiansPerSecond)));

        // Correct our Kalman filter's state vector estimate with encoder data.
        loop.correct(VecBuilder.fill(getCurrentVelocity().in(RadiansPerSecond)));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state without Kalman filter.
        loop.predict(TIME_DIFFERENCE);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = loop.getU(0);
        motor.setVoltage(nextVoltage);
    }

    public void setGoal(Measure<Velocity<Angle>> targetVelocity) {
        goal = targetVelocity;
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return RPM.of(encoder.getVelocity());
    }
}

