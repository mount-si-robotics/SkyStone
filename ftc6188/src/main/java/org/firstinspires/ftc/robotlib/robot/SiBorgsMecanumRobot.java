package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.armsystem.FieldGoalArmSystem;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.motor.CalculatedVelocityMotor;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;
import org.firstinspires.ftc.robotlib.servo.LinkedServo;
import org.firstinspires.ftc.robotlib.sound.BasicSound;
import org.jetbrains.annotations.NotNull;

public class SiBorgsMecanumRobot
{
    // Drive motors
    private CalculatedVelocityMotor driveFrontLeft;
    private CalculatedVelocityMotor driveFrontRight;
    private CalculatedVelocityMotor driveRearLeft;
    private CalculatedVelocityMotor driveRearRight;
    private CalculatedVelocityMotor[] driveMotorList;

    // Drive constants
    private static final double WHEEL_RADIUS_IN = 2;
    private static final double MOTOR_TO_WHEEL_RATIO = 2;

    // Arm motors
    private LimitedMotor armVerticalSlide;
    private LimitedMotor armHorizontalSlide;

    // Arm constants
    private static final int[] VERTICAL_LIMIT = {0, 1270};
    private static final int[] HORIZONTAL_LIMIT = {-1400, -400};

    // Servo motors
    private Servo servoClawLeft;
    private Servo servoClawRight;
    public Servo armGripSlide;

    public LinkedServo platformServo;

    // Sound objects
    public BasicSound sirenSound;

    // Telemetry reference
    private final Telemetry telemetry;

    // Movement systems (drivetrain/arm system)
    public MecanumDrivetrain drivetrain;
    public FieldGoalArmSystem crane;

    // Class constructor with full systems initialization
    public SiBorgsMecanumRobot (@NotNull HardwareMap hwMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        // Drive motors init
        driveFrontLeft = new CalculatedVelocityMotor(hwMap.get(DcMotor.class, "driveFrontLeft"));
        driveFrontRight = new CalculatedVelocityMotor(hwMap.get(DcMotor.class, "driveFrontRight"));
        driveRearRight = new CalculatedVelocityMotor(hwMap.get(DcMotor.class, "driveRearRight"));
        driveRearLeft = new CalculatedVelocityMotor(hwMap.get(DcMotor.class, "driveRearLeft"));
        driveMotorList = new CalculatedVelocityMotor[] {driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        // Since all drive motors are considered the same a loop is used
        for (DcMotor motor : driveMotorList)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Arm motors init
        armVerticalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armVerticalSlide"), VERTICAL_LIMIT[0], VERTICAL_LIMIT[1]);
        armHorizontalSlide = new LimitedMotor(hwMap.get(DcMotor.class, "armHorizontalSlide"), HORIZONTAL_LIMIT[0], HORIZONTAL_LIMIT[1]);

        armVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armVerticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armHorizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armVerticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        armHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        armVerticalSlide.setLimited(true);
        armHorizontalSlide.setLimited(true);

        // Servos init
        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");
        armGripSlide = hwMap.get(Servo.class, "armGripSlide");

        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.FORWARD);
        armGripSlide.setDirection(Servo.Direction.FORWARD);

        servoClawLeft.setPosition(0);
        servoClawRight.setPosition(0);
        armGripSlide.setPosition(1);

        platformServo = new LinkedServo(servoClawLeft, servoClawRight);

        // Sounds init
        sirenSound = new BasicSound("police_siren", hwMap);

        // Systems init
        drivetrain = new MecanumDrivetrain(driveMotorList, WHEEL_RADIUS_IN, MOTOR_TO_WHEEL_RATIO);
        crane = new FieldGoalArmSystem(armVerticalSlide, armHorizontalSlide);
    }

    // Telemetry command useful for drivers
    public void driverTelemetry()
    {
        telemetry.addData("> Drive Info", "-----");
        telemetry.addData("Half Power Mode\t(G1-RStickButton)", drivetrain.getLowPower());
        telemetry.addData("Course Degrees\t(G1-RStick)", drivetrain.getCourse());
        telemetry.addData("Velocity\t(G1-RStick)", drivetrain.getVelocity());
        telemetry.addData("Rotation\t(G1-LStick)", drivetrain.getRotation());

        telemetry.addData("> Arm Info", "Limited\t(G2-B)? " + armVerticalSlide.isLimited());
        telemetry.addData("Vertical Position\t(G2-LStickY)", armVerticalSlide.getCurrentPosition());
        telemetry.addData("Horizontal Position\t(G2-RStickY", armHorizontalSlide.getCurrentPosition());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Platform Servos Pos\t(G2-DpadUp/DpadDown)", platformServo.getPosition());
        telemetry.addData("Platform Claw Left\t(^)", servoClawLeft.getPosition());
        telemetry.addData("Platform Claw Right\t(^)", servoClawRight.getPosition());
        telemetry.addData("Arm Grip Slide\t(G2-Y/A)", armGripSlide.getPosition());

        telemetry.update();
    }

    // Full output of all basic functions for debugging
    public void informationTelemetry()
    {
        telemetry.addData("> Target Positions", "-----");
        telemetry.addData("WheelTarget FL", drivetrain.wheelTargetPositions[0]);
        telemetry.addData("WheelTarget FR", drivetrain.wheelTargetPositions[1]);
        telemetry.addData("WheelTarget RL", drivetrain.wheelTargetPositions[2]);
        telemetry.addData("WheelTarget RR", drivetrain.wheelTargetPositions[3]);
        telemetry.addData("Distance Target", drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", drivetrain.motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", drivetrain.motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", drivetrain.motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", drivetrain.motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos Percent", drivetrain.getCurrentPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition() * drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Motor Revs Per Sec", "-----");
        telemetry.addData("Speed FL", drivetrain.motorList[0].getRevolutionsPerSecond());
        telemetry.addData("Speed FR", drivetrain.motorList[1].getRevolutionsPerSecond());
        telemetry.addData("Speed RL", drivetrain.motorList[2].getRevolutionsPerSecond());
        telemetry.addData("Speed RR", drivetrain.motorList[3].getRevolutionsPerSecond());

        telemetry.addData("> Is Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isBusy());

        telemetry.addData("> Is Encoder Busy", "-----");
        telemetry.addData("FL", drivetrain.motorList[0].isEncoderBusy());
        telemetry.addData("FR", drivetrain.motorList[1].isEncoderBusy());
        telemetry.addData("RL", drivetrain.motorList[2].isEncoderBusy());
        telemetry.addData("RR", drivetrain.motorList[3].isEncoderBusy());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());
        telemetry.addData("Is Pos", drivetrain.isPositioning());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Servo Pos", "One: " + platformServo.getServoOne().getPosition() + " Two: " + platformServo.getServoTwo().getPosition());
        telemetry.addData("Servo Pos2", "One: " + servoClawLeft.getPosition() + " Two: " + servoClawRight.getPosition());
        telemetry.addData("Linked Pos", platformServo.getPosition());

        telemetry.update();
    }

    // Full information output plus an optional status
    public void informationTelemetry(String programStatus)
    {
        telemetry.addData("> Prog Status", programStatus);
        informationTelemetry();
    }
}