package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.drivetrain.BasicOdometricalMecanumDrivetrain;

public class BasicOdometricalMecanumHardwareMap extends MecanumHardwareMap {
    // REV IMU
    public BNO055IMUImpl imu;
    public BasicOdometricalMecanumDrivetrain drivetrain;

    /**
     * Creates an odometrical mecanum hardware map
     * @param hwMap FTC hardware map
     */
    public BasicOdometricalMecanumHardwareMap(HardwareMap hwMap) {
        super(hwMap);

        // Configure the REV HUB IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMUImpl.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());

        drivetrain = new BasicOdometricalMecanumDrivetrain(motorList, imu);
    }
}
