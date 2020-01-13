package org.firstinspires.ftc.robotlib.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.state.AutoDirection;

public abstract class BasicTimeBasedAuto extends LinearOpMode
{
    protected ElapsedTime timer;
    final double VELOCITY;
    final double COMMAND_SPACE;

    BasicTimeBasedAuto(double velocity, double commandSpace)
    {
        this.VELOCITY = velocity;
        this.COMMAND_SPACE = commandSpace;
        timer = new ElapsedTime();
    }

    @Override
    public void runOpMode() throws InterruptedException { }

    public abstract void position(AutoDirection course, double time);
}
