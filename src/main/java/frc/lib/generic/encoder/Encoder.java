package frc.lib.generic.encoder;

public interface Encoder {

    void reset();

    void getPosition();
    void getVelocity();

    boolean configure();

}
