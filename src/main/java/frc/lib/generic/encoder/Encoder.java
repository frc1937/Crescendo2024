package frc.lib.generic.encoder;

public interface Encoder {

    void reset();

    double getEncoderPosition();
    double getEncoderVelocity();

    boolean configure();

}
