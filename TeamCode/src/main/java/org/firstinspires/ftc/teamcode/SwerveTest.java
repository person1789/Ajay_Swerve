package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWMap;
import org.firstinspires.ftc.teamcode.Swerve.DT;

@TeleOp

public class SwerveTest extends LinearOpMode {

    private ElapsedTime timer;
    private double loopTime = 0;

    private final HWMap robot = HWMap.getInstance();
    private DT swerve;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
