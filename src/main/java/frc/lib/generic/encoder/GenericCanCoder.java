//package frc.lib.generic.encoder;
//
//import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.hardware.CANcoder;
//
///**
// * Wrapper class for the CAN encoder.
// * Verify its setup is correct via this:
// * <a href="https://store.ctr-electronics.com/content/user-manual/CANCoder%20User">CTRE CANcoder PDF</a>'s%20Guide.pdf
// */
//public class GenericCanCoder  extends CANcoder implements Encoder {
//
//    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
//
//    private StatusSignal<Double> positionSignal, velocitySignal;
//
//    public GenericCanCoder(int canCoderID) {
//        super(canCoderID);
//
//        positionSignal = super.getPosition().clone();
//        velocitySignal = super.getVelocity().clone();
//    }
//
//    @Override
//    public double getEncoderPosition() {
//        return positionSignal.refresh().getValue();
//    }
//
//    @Override
//    public double getEncoderVelocity() {
//        return velocitySignal.refresh().getValue();
//    }
//
//    @Override
//    public boolean configure() {
//        canCoderConfig.MagnetSensor.MagnetOffset;
//        canCoderConfig.MagnetSensor.SensorDirection;
//        canCoderConfig.MagnetSensor.AbsoluteSensorRange;
//
//        return false;
//    }
//}
////todo: Implement cancoder