package org.firstinspires.ftc.opmodes.auto;

import org.firstinspires.ftc.robotlib.state.Alliance;
import org.firstinspires.ftc.robotlib.state.Course;

public class BridgeParkAutonomous extends AutonomousBase {
    @Override
    public void initializeOpmode() {

    }

    @Override
    public void startOpmode() {
        while (elapsedTime.seconds() < 20);

        robot.simpleMove(Course.BACKWARD, 0.7, 0, 1);
        robot.simpleMove(alliance == Alliance.BLUE ? Course.RIGHT : Course.LEFT, 0.7, 0, 20);
    }

    @Override
    public void endOpmode() {

    }
}
