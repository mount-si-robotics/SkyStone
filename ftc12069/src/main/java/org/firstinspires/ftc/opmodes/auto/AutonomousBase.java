package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.state.Alliance;

public abstract class AutonomousBase extends LinearOpMode {
    protected Alliance alliance = Alliance.BLUE;
    protected ElapsedTime elapsedTime = new ElapsedTime();
    protected AutonomousRobot robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot = new AutonomousRobot(this.hardwareMap, alliance, this, elapsedTime);
        robot.init();

        initializeOpmode();
        waitForStart();
        startOpmode();
        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
        endOpmode();
    }

    /**
     * Initializes the opmode
     * Ran before the game starts
     */
    public abstract void initializeOpmode();

    /**
     * Ran after the game starts and before the game loop begins
     */
    public abstract void startOpmode();

    /**
     * Ran once the game has ended
     */
    public abstract void endOpmode();
}
