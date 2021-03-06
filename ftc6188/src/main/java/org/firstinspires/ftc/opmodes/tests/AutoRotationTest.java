package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumIMURobot;

@Disabled
@Autonomous(name="Auto Rotation Test", group="Test")
public class AutoRotationTest extends LinearOpMode
{
    // Robot reference
    private SiBorgsMecanumIMURobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumIMURobot(this.hardwareMap, this.telemetry);

        while (!isStarted()) { robot.angleTelemetry(); }

        robot.drivetrain.autoRotate(90, 1);
        sleep(5000);
        robot.drivetrain.autoRotate(180, 1);
        sleep(5000);
        robot.drivetrain.autoRotate(270, 1);
        sleep(5000);
        robot.drivetrain.autoRotate(0, 1);
        sleep(5000);
    }

    @Override
    public void internalPostLoop()
    {
        try
        {
            robot.angleTelemetry();
        }
        catch (NullPointerException ignored) { }
    }
}
