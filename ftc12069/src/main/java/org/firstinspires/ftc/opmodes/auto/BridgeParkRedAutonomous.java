package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.state.Alliance;
import org.firstinspires.ftc.robotlib.state.Course;

@Autonomous(name="Bridge Park Autonomous Red", group="auto")
public class BridgeParkRedAutonomous extends BridgeParkAutonomous {
    {
        alliance = Alliance.RED;
    }
}
