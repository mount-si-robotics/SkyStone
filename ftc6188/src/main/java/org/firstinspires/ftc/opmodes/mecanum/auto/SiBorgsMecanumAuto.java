package org.firstinspires.ftc.opmodes.mecanum.auto;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Mecanum Auto V-CompetitionReady", group="AutoComp")
public class SiBorgsMecanumAuto extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Fields
    private static final double VELOCITY = 0.5;

    // Buttons
    private Button capstoneOpen;
    private Button capstoneClose;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        elapsedTime = new ElapsedTime();
        capstoneOpen = new Button();
        capstoneClose = new Button();

        while (!isStarted())
        {
            capstoneOpen.input(gamepad1.dpad_up || gamepad2.dpad_down || gamepad1.y || gamepad2.y);
            capstoneClose.input(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.a || gamepad2.a);

            if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
            else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

            telemetry.addData("ADD CAPSTONE TO SERVO ", robot.armGripSlide.getState());
            telemetry.update();
        }
        telemetry.addData("AUTO START", elapsedTime.seconds());
        telemetry.update();

        robot.drivetrain.autoPosition(0, 48, VELOCITY, 0);

        requestOpModeStop();
    }
}

enum AutoDirection
{
    FRONT(Math.atan2(1, 0) - Math.PI/2),
    LEFT(Math.atan2(0, -1) - Math.PI/2),
    RIGHT(Math.atan2(0, 1) - Math.PI/2),
    REAR(Math.atan2(1, 0) - Math.PI/2);

    private double angle;

    public double getAngle() { return angle; }

    AutoDirection(double angle) { this.angle = angle; }
}
