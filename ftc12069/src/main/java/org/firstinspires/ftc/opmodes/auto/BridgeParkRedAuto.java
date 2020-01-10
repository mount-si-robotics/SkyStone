package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.state.Alliance;
import org.firstinspires.ftc.robotlib.state.Course;

@Autonomous(name="Bridge Park Autonomous Red", group="auto")
public class BridgeParkRedAuto extends LinearOpMode {
    private Alliance alliance = Alliance.RED;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private AutonomousRobot robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new AutonomousRobot(this.hardwareMap, alliance, telemetry, elapsedTime);
        robot.init();

        elapsedTime.reset();
        // Wait for the game to start
        waitForStart();
        while (elapsedTime.seconds() < 20);

        robot.simpleMove(Course.BACKWARD, 0.7, 0, 1);
        robot.simpleMove(Course.LEFT, 0.7, 0, 20);
    }
}
