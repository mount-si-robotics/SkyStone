package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotlib.state.Alliance;

@Autonomous(name="Foundation Move Blue", group="auto")
public class FoundationDeliverBlueAutonomous extends FoundationDeliverAutonomous {
    {
        alliance = Alliance.BLUE;
    }
}
