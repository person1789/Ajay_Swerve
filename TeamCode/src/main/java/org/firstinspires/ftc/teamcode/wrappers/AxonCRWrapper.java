package org.firstinspires.ftc.teamcode.wrappers;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class AxonCRWrapper {
    private CRServo axon;
    private Motor.Encoder encoder;
    private double lastReadPosition;
    private double sign = 1;
    private double encoderOffset;
    private double inverseEncoderOffset;
    public AxonCRWrapper(CRServo axon, Motor.Encoder encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset) {
        this.axon = axon;
        this.encoder = encoder;
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            inverseEncoderOffset = 360;
        }

        this.encoderOffset = encoderOffset;
    }

    public void set(double power) {
        axon.set(power * sign);
    }

    public double get() {
        return axon.get() * sign;
    }

    public double readPos() {

        lastReadPosition = (Math.abs(inverseEncoderOffset - encoder.getPosition() + encoderOffset)) % 360;

        return lastReadPosition;
    }

    public double readRawPos() {
        return encoder.getPosition();
    }

    public double getLastReadPos() {
        return lastReadPosition;
    }

    public void setEncoderOffset(double offset) {
        this.encoderOffset = offset;
    }

}
