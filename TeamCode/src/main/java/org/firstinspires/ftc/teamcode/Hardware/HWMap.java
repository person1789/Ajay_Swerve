package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class HWMap {
    public static IMU imu;
    public static double imuAngle;
    public final VoltageSensor voltageSensor;

    public final DcMotorEx frontRightMotor;
    public final DcMotorEx frontLeftMotor;
    public final DcMotorEx backLeftMotor;
    public final DcMotorEx backRightMotor;

    public final CRServo frontRightServo;
    public final CRServo frontLeftServo;
    public final CRServo backLeftServo;
    public final CRServo backRightServo;

    public final AnalogInput frontRightEncoder;
    public final AnalogInput frontLeftEncoder;
    public final AnalogInput backLeftEncoder;
    public final AnalogInput backRightEncoder;

    public static boolean initialized = false;
    List<LynxModule> hubs;

    public HWMap(HardwareMap hardwareMap, boolean isAuto) {
        hubs = hardwareMap.getAll(LynxModule.class);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRM");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLM");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLM");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BRM");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        frontRightServo =  hardwareMap.get(CRServo.class, "FRS");
        frontLeftServo =  hardwareMap.get(CRServo.class, "FLS");
        backLeftServo =  hardwareMap.get(CRServo.class, "BLS");
        backRightServo =  hardwareMap.get(CRServo.class, "BRS");

        frontRightEncoder = hardwareMap.get(AnalogInput.class, "FRE");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "FLE");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "BLE");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "BRE");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        initialized = true;
        return imuAngle;
    }

    public static double getImuAngle() {
        return imuAngle;
    }

    public static double initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
        initialized = true;
        return imuAngle;
    }



    public AnalogInput getFrontRightEncoder(){
        return frontRightEncoder;
    }

    public AnalogInput getFrontLeftEncoder(){
        return frontLeftEncoder;
    }

    public AnalogInput getBackLeftEncoder(){
        return backLeftEncoder;
    }

    public AnalogInput getBackRightEncoder() {
        return backRightEncoder;
    }
}
