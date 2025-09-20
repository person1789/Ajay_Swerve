package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Geo.MathUtils.norm;
import static org.firstinspires.ftc.teamcode.Geo.MathUtils.normDelta;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;


import java.lang.Math;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }
    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;
    public double getCurrentPosition() {
        double pos = norm((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(normDelta(pastPosition)) > 0.1 || Math.abs(normDelta(pos)) < 1)
            pastPosition = pos;
        return pastPosition;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}
