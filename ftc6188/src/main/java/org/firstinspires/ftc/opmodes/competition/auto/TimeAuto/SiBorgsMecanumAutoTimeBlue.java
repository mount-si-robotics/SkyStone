package org.firstinspires.ftc.opmodes.competition.auto.TimeAuto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@Autonomous(name="BLUE Comp Auto", group="CompAuto")
public class SiBorgsMecanumAutoTimeBlue extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;

    // Fields
    private static final double VELOCITY = 0.5;

    // Buttons
    private Button capstoneOpen;
    private Button capstoneClose;

    // End Behavior
    private Button endToggle;
    private ToggleBoolean parkFront;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        robot.changeBackgroundColor(Color.BLUE);

        capstoneOpen = new Button();
        capstoneClose = new Button();

        /** Before the auto period starts the drivers should load a capstone into the arm **/
        while (!isStarted())
        {
            capstoneOpen.input(gamepad1.dpad_up || gamepad2.dpad_down || gamepad1.y || gamepad2.y);
            capstoneClose.input(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.a || gamepad2.a);

            if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
            else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

            robot.armCrane.setVerticalPower(-gamepad1.left_stick_y);
            robot.armCrane.setHorizontalPower(gamepad1.right_stick_y);

            updateEndToggle();
            telemetry.addData("ADD CAPSTONE TO SERVO (G1-DPAD) + (G1-LStick)", robot.armGripSlide.getState()); telemetry.update();
        }
        telemetry.addData("START OF AUTO PERIOD", ""); telemetry.update();
        /** Auto period now starts **/

        /** AUTO COMMANDS **/
        robot.drivetrain.autoPositionByTime(AutoDirection.RIGHT, 0.75, VELOCITY);
        sleep(250);
        robot.drivetrain.autoPositionByTime(AutoDirection.FRONT, 3.5, VELOCITY/2);
        robot.platformServo.setPosition(ServoState.DOWN);
        sleep(500);
        robot.drivetrain.autoPositionByTime(AutoDirection.REAR, 2, VELOCITY);
        robot.platformServo.setPosition(ServoState.UP);
        sleep(500);
        robot.drivetrain.autoPositionByTime(AutoDirection.LEFT, 1.75, VELOCITY);
        robot.drivetrain.autoPositionByTime(AutoDirection.FRONT, 0.125, VELOCITY);
        robot.drivetrain.autoPositionByTime(AutoDirection.LEFT, 0.3, VELOCITY);
        sleep(250);
        robot.drivetrain.autoPositionByTime(AutoDirection.FRONT, 0.6, VELOCITY);
        sleep(250);
        robot.drivetrain.autoPositionByTime(AutoDirection.RIGHT, 0.8, VELOCITY);
        sleep(250);

        if (parkFront.output())
        {
            robot.drivetrain.autoPositionByTime(AutoDirection.FRONT, 0.35, VELOCITY);
            sleep(250);
            robot.drivetrain.autoPositionByTime(AutoDirection.LEFT, 1.9, VELOCITY);
        }
        else
        {
            robot.drivetrain.autoPositionByTime(AutoDirection.REAR, 0.7, VELOCITY);
            sleep(250);
            robot.drivetrain.autoPositionByTime(AutoDirection.LEFT, 2, VELOCITY);
            sleep(250);
            robot.drivetrain.autoPositionByTime(AutoDirection.REAR, 0.5, VELOCITY);
        }
        sleep(1000);
        requestOpModeStop();
    }

    public void updateEndToggle()
    {
        if (endToggle.onPress())
        { parkFront.toggle(); }

        telemetry.addData("End motion: ", parkFront.output() ? "Inside" : "Outside");
    }
}
