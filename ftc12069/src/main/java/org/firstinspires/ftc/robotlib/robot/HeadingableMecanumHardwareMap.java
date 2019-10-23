package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.controller.ErrorTimeThresholdFinishingAlgorithim;
import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;
import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.drivetrain.HeadingableMecanumDrivetrain;
import org.firstinspires.ftc.robotlib.sensor.IntegratingGyroscopeSensor;

public class HeadingableMecanumHardwareMap extends MecanumHardwareMap
{
    public BNO055IMUImpl imu;

    public FinishableIntegratedController controller;
    public HeadingableMecanumDrivetrain drivetrain;

    public HeadingableMecanumHardwareMap(HardwareMap hwMap)
    {
        super(hwMap);

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
    }
}