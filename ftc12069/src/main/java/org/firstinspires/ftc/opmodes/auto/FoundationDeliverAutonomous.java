package org.firstinspires.ftc.opmodes.auto;

import org.firstinspires.ftc.robotlib.state.Course;

public class FoundationDeliverAutonomous extends AutonomousBase {
    @Override
    public void initializeOpmode() {

    }

    @Override
    public void startOpmode() {
        robot.move(Course.FORWARD, 0.2, null, 4);
        robot.move(Course.LEFT, 0.5, null, 70);
        robot.move(Course.FORWARD, 0.5, null, 35);
        robot.hardware.platformServoLeft.setPosition(0.0);
        robot.hardware.platformServoRight.setPosition(1.0);
        sleep(2000);
        //robot.move(Course.BACKWARD, 0.5, null, 25);
        robot.hardware.platformServoLeft.setPosition(1.0);
        robot.hardware.platformServoRight.setPosition(0.0);
        sleep(2000);
        //robot.turn(-90);
        robot.simpleMove(Course.RIGHT, 0.5, 0, 50);
        robot.simpleMove(Course.FORWARD, 0.5, 0, 20);
        robot.simpleMove(Course.LEFT, 0.5, 0, 10);
        robot.simpleMove(Course.BACKWARD, 0.5, 0, 5);
    }

    @Override
    public void endOpmode() {

    }
}
