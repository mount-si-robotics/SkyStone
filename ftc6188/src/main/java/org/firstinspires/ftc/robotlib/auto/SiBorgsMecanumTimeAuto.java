package org.firstinspires.ftc.robotlib.auto;

import android.graphics.Color;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class SiBorgsMecanumTimeAuto extends BasicTimeBasedAuto
{
    // Robot ref
    protected SiBorgsMecanumRobot robot;

    // Buttons
    private Button capstoneOpen;
    private Button capstoneClose;
    private Button allianceToggler;
    private String definition = "DEF";

    // End Behavior
    private Button endToggler;
    protected ToggleBoolean parkInside; // true if parking on the inside
    private ToggleBoolean isBlue; // write all programs from the perspective of BLUE

    public SiBorgsMecanumTimeAuto()
    {
        super(0.5, 500);
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);

        capstoneOpen = new Button();
        capstoneClose = new Button();
        allianceToggler = new Button();

        endToggler = new Button();
        parkInside = new ToggleBoolean(true);
        isBlue = new ToggleBoolean(true);
    }

    public void init(String definition)
    {
        this.definition = definition;
        updateBackgroundColor();
    }

    protected void initLoop()
    {
        /** Before the auto period starts the drivers should load a capstone into the arm **/

        capstoneOpen.input(gamepad1.a);
        capstoneClose.input(gamepad1.y);
        allianceToggler.input(gamepad1.x);
        endToggler.input(gamepad1.b);

        if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
        else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

        robot.armCrane.setVerticalPower(-gamepad1.left_stick_y);
        robot.armCrane.setHorizontalPower(gamepad1.right_stick_y);

        if (endToggler.onPress())
        { parkInside.toggle(); }

        if (allianceToggler.onPress())
        { isBlue.toggle(); }

        updateBackgroundColor();

        telemetry.addData("Alliance(X): ", isBlue.output() ? "Blue" : "Red");
        telemetry.addData("End motion(B): ", parkInside.output() ? "Inside" : "Outside");
        telemetry.addData("Program Definition: ", definition);
        telemetry.addData("ADD CAPSTONE TO SERVO (G1-DPAD) + (G1-LStick): ", robot.armGripSlide.getState()); telemetry.update();
        telemetry.update();
    }

    private void updateBackgroundColor()
    {
        if (isBlue.output())
        { robot.changeBackgroundColor(Color.BLUE); }
        else
        { robot.changeBackgroundColor(Color.RED); }
    }

    @Override
    public void position(AutoDirection course, double time)
    {
        if ((course == AutoDirection.LEFT || course == AutoDirection.RIGHT) && !isBlue.output())
        {
            if (course == AutoDirection.LEFT)
            { robot.drivetrain.autoPositionByTime(AutoDirection.RIGHT, time, super.VELOCITY); }
            else
            { robot.drivetrain.autoPositionByTime(AutoDirection.LEFT, time, super.VELOCITY); }
        }
        robot.drivetrain.autoPositionByTime(course, time, super.VELOCITY);
        sleep((long) super.COMMAND_SPACE);
    }
}
