package org.firstinspires.ftc.robotlib.autonomous;

import org.firstinspires.ftc.robotlib.state.OrientationState;

/**
 * Old class for managing heading during movement
 * @deprecated
 */
public class HeadingManager {
    private OrientationState orientationState = OrientationState.STATIONERY;
    private double targetHeading = 0;
    private double initialHeading = 0;
    private double currentHeading = 0;

    public double getInitialHeading() {
        return initialHeading;
    }

    public void setInitialHeading(double initialHeading) {
        this.initialHeading = initialHeading;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getCurrentHeading() {
        return currentHeading;
    }

    public void setCurrentHeading(double currentHeading) {
        this.currentHeading = currentHeading;
    }

    public OrientationState getState() {
        return orientationState;
    }
}
