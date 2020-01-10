package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotlib.controller.ErrorTimeThresholdFinishingAlgorithim;
import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;
import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.drivetrain.HeadingableMecanumDrivetrain;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.sensor.IntegratingGyroscopeSensor;
import org.firstinspires.ftc.robotlib.servo.ServoManager;

public class HeadingableMecanumHardwareMap
{
    public BNO055IMUImpl imu;
    public HardwareMap internalHardwareMap;

    // Motors in Mecanum robot
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveRearRight;
    private DcMotor driveRearLeft;

    // Servos in Mecanum Robot
    private Servo servoClaw;

    // Camera
    public WebcamName webcamName;

    //public BNO055IMU imu;

    public ServoManager servoManager;

    public final double wheelRadius = 4; //inches
    private static final double wheelToMotorRatio = 1.0/1.0;

    public final double motorTicksPerInch;

    public DcMotor[] motorList;


    public FinishableIntegratedController controller;
    public HeadingableMecanumDrivetrain drivetrain;

    /**
     * Creates a mecanum hardware map from the FTC given hardware map
     * @param hwMap FTC hardware map
     */
    public HeadingableMecanumHardwareMap(HardwareMap hwMap)
    {
        this.internalHardwareMap = hwMap;


        driveFrontLeft = hwMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hwMap.get(DcMotor.class, "driveFrontRight");
        driveRearRight = hwMap.get(DcMotor.class, "driveRearRight");
        driveRearLeft = hwMap.get(DcMotor.class, "driveRearLeft");

        motorList = new DcMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        imu = hwMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters paramaters = new BNO055IMU.Parameters();
        paramaters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        paramaters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        paramaters.loggingEnabled = true;
        paramaters.loggingTag = "IMU";
        paramaters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(paramaters);

        PIDController pid = new PIDController(1.5, 0.05, 0);
        pid.setMaxErrorForIntegral(0.002);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithim(Math.PI/50, 1));
        drivetrain = new HeadingableMecanumDrivetrain(motorList, controller);

        //servoManager = new ServoManager(new Servo[]{servoClaw});
        motorTicksPerInch = drivetrain.getTicksPerInch(wheelRadius, wheelToMotorRatio);
    }

}
