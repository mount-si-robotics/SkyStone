package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotlib.state.Alliance;

@Autonomous(name="Foundation Move Red", group="auto")
public class FoundationDeliverRedAutonomous extends FoundationDeliverAutonomous {
    {
        alliance = Alliance.RED;
    }
}
