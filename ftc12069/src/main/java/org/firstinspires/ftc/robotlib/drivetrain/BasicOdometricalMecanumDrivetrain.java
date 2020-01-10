package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BasicOdometricalMecanumDrivetrain extends MecanumDrivetrain implements Odometrical {
    private IntegratingGyroscope gyroscope;
    private double targetHeading = 0;
    private double headingTolerance = 3;

    public BasicOdometricalMecanumDrivetrain(EncoderMotor[] motors, IntegratingGyroscope gyroscope) {
        super(motors);
        this.gyroscope = gyroscope;
    }

    public IntegratingGyroscope getGyroscope() {
        return gyroscope;
    }

    @Override
    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    @Override
    public double getCurrentHeading() {
        return getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
    }

    @Override
    public double getTargetHeading() {
        return this.targetHeading;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    @Override
    public void updateHeading() {
        this.setMotorPowers(this.getWheelRotationValues(0.5));
    }

    @Override
    public void rotate() {
        while (isRotating()) {
            updateHeading();
        }
        finishRotating();
    }

    @Override
    public boolean isRotating() {
        double currentHeading = Math.abs(getCurrentHeading());
        double targetHeading = Math.abs(getTargetHeading());
        return currentHeading > targetHeading - getHeadingTolerance() && currentHeading < targetHeading + getHeadingTolerance();
    }

    @Override
    public void finishRotating() {

    }
}
