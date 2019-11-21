package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.firstinspires.ftc.robotlib.sound.BasicSound;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

@TeleOp(name="Mecanum TeleOp V-Final", group="Tele")
public class MecanumTeleOp extends OpMode
{
    // TeleOp specific variables
    private MecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Buttons and toggles
    private ToggleBoolean driverTwoBrakes;  //freezes robot in place for stacking, prevents stick bumping from driver one
    private Button playSound;
    private Button servosUp;
    private Button servosDown;

    // Sound players
    private BasicSound basicSound;

    @Override
    public void init()
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, true);
        elapsedTime = new ElapsedTime();

        driverTwoBrakes = new ToggleBoolean(false);
        playSound = new Button();
        servosUp = new Button();
        servosDown = new Button();

        basicSound = new BasicSound("police_siren", this.hardwareMap, 0, true);
    }

    @Override
    public void init_loop()
    {
        telemetry.addData("Status", "Init Loop");
        telemetry.update();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        //DRIVER ONE
        // both convert sticks into vectors and take two different readings from the resulting vector
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        // gamepad 1 inputs
        robot.drivetrain.lowPowerInput(gamepad1.right_stick_button);
        playSound.input(gamepad1.x);
        servosDown.input(gamepad1.dpad_down);

        // movement controls
        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity * (driverTwoBrakes.output() ? 0 : 1));
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        // sound
        if (playSound.onRelease()) { basicSound.toggleSound(); }


        //DRIVER TWO
        // gamepad 2 inputs
        servosUp.input(gamepad1.dpad_up);

        //arm movement to be added later
        driverTwoBrakes.input(gamepad2.left_bumper);

        // servo controls
        if (servosUp.onPress()){robot.platformServos.setPosition(1); }
        else if (servosDown.onPress()){robot.platformServos.setPosition(0);}


        //TELEMETRY
        robot.informationUpdate();
    }

    @Override
    public void stop()
    {
        basicSound.stopSound(); // very important otherwise it will keep playing forever

        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
