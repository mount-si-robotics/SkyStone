package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

/*
A StateServo is a servo whose position can be set by specifying an enum state
 */
public class StateServo extends ModifiedServo
{
    private double[] positions;
    private ServoState servoState;

    public StateServo(Servo servo, double stowedPosition, double upPosition, double downPosition)
    {
        super(servo);

        positions = new double[] {stowedPosition, upPosition, downPosition};
        servoState = ServoState.STOWED;
    }

    public StateServo(Servo servo) { this(servo, 0, 0, 1); }

    public void setPosition(ServoState servoState)
    {
        if (servoState != ServoState.UNKNOWN)
        {
            setPosition(positions[servoState.getStateLevel()]);
            this.servoState = servoState;
        }
    }

    @Override
    public void setPosition(double position)
    {
        this.servo.setPosition(position);
        servoState = ServoState.UNKNOWN;
    }

    public ServoState getState() { return servoState; }

    private void setPositions(double[] positions) { this.positions = positions; }

    public void setPositions(double stowedPosition, double upPosition, double downPosition) { setPositions(new double[] {stowedPosition, upPosition, downPosition});}

    public double[] getPositions() { return positions; }

    public Servo getServo() { return servo; }
}
