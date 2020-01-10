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
        robot.move(Course.FORWARD, 9.5, null, 45);
        robot.hardware.platformServoLeft.setPosition(1.0);
        robot.hardware.platformServoRight.setPosition(0.0);
        robot.move(Course.BACKWARD, 0.5, null, 45);
        robot.move(Course.RIGHT, 0.5, null, 60);
    }

    @Override
    public void endOpmode() {

    }
}
