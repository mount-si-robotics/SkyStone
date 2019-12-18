package org.firstinspires.ftc.opmodes.competition.auto.TimeAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotlib.auto.SiBorgsMecanumTimeAuto;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Build Side | Platform + Bridge Park", group="CompAuto")
public class BasicAutoBuildSideFull extends SiBorgsMecanumTimeAuto
{
    @Override
    public void runOpMode()
    {
        init("Platform + Bridge Park");
        while (!isStarted()) { initLoop(); }

        // do auto bs here
        super.position(AutoDirection.RIGHT, 0.75);
        super.position(AutoDirection.FRONT, 3.5);
        super.robot.platformServo.setPosition(ServoState.DOWN);

        super.position(AutoDirection.REAR, 2);
        super.robot.platformServo.setPosition(ServoState.UP);

        super.position(AutoDirection.LEFT, 1.75);
        super.position(AutoDirection.FRONT, .125);
        super.position(AutoDirection.LEFT, .3);
        super.position(AutoDirection.FRONT, 0.6);
        super.position(AutoDirection.RIGHT, 0.8);

        if (super.parkInside.output())
        {
            super.position(AutoDirection.FRONT, 0.35);
            super.position(AutoDirection.LEFT, 1.9);
        }
        else
        {
            super.position(AutoDirection.REAR, 0.7);
            super.position(AutoDirection.LEFT, 2);
            super.position(AutoDirection.REAR, 0.5);
        }
    }
}
