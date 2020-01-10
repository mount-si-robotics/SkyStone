/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.autonomous.HeadingManager;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.robot.HeadingableMecanumHardwareMap;
import org.firstinspires.ftc.robotlib.autonomous.HeadingableAutonomousRobot;
import org.firstinspires.ftc.robotlib.state.Alliance;
@Autonomous(name = "Calibrate", group = "auto")
public class Calibrate extends LinearOpMode {
    //
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    //Calculate encoder conversion
    private Integer cpr = 28; //counts per rotation
    private Integer gearratio = 2 / 1;
    private Double diameter = 4.125;
    private Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    private Double bias;//adjust until your robot goes 20 inches
    //
    private Double conversion;
    //
    public HeadingableAutonomousRobot robot;
    {
        conversion = cpi * bias;
    }

    public Calibrate() {
        bias = 0.8;
    }

    //
    public void runOpMode() {
        //
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        //
        waitForStartify();
        //
        moveToPosition();//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    private void moveToPosition() {
        //
        int move1 = (int) (Math.round(((double) 20 - 5) * conversion));
        int movefl2 = frontleft.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movefr2 = frontright.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movebl2 = backleft.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movebr2 = backright.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        //
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move1);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move1);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move1);
        backright.setTargetPosition(backright.getCurrentPosition() + move1);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(0.2);
        frontright.setPower(0.2);
        backleft.setPower(0.2);
        backright.setPower(0.2);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        //
        frontleft.setTargetPosition(movefl2);
        frontright.setTargetPosition(movefr2);
        backleft.setTargetPosition(movebl2);
        backright.setTargetPosition(movebr2);
        //
        frontleft.setPower(.1);
        frontright.setPower(.1);
        backleft.setPower(.1);
        backright.setPower(.1);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    private void waitForStartify() {
        waitForStart();
    }

class CalibrateVuforia {
    private Alliance alliance;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private AutonomousRobot robot;

    CalibrateVuforia(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    /**
     * Ran before the game starts
     */
    void init() {
        // Initialize robot
        telemetry.addData("Status", "Initialized");
        robot = new AutonomousRobot(this.hardwareMap, alliance, telemetry);
        robot.init();
    }

    /**
     * Ran after the game starts and before the game loop begins
     */
    void start() {
        elapsedTime.reset();

        // Enable Tracking
        robot.trackables.activate();
    }

    /**
     * Ran once the game has ended
     */
    void end() {
        // Disable Tracking when we are done
        robot.trackables.deactivate();
    }

    /**
     * Game Loop Method (runs until stop is requested)
     * @return true - keep looping | false - stop looping
     */
    boolean loop() {
        /*if (elapsedTime.seconds() > 25) {
            robot.parkUnderBridge();
            return false;
        }*/

        robot.scan();

        telemetry.addData("Elapsed Time", elapsedTime.seconds() + " seconds");
        // Provide feedback as to where the robot is located (if we know).
        if (robot.isTrackableVisible()) {
            if (robot.isLocationKnown()) {
                // express position (translation) of robot in inches.
                Point3D position = robot.getPosition();
                telemetry.addData("Position (inch)", "{X, Y, Z} = %.1f, %.1f, %.1f", position.x, position.y, position.z);

                // express the orientation of the robot in degrees.
                Orientation orientation = robot.getOrientation();
                telemetry.addData("Orientation (deg)", "{Heading, Roll, Pitch} = %.0f, %.0f, %.0f", orientation.thirdAngle, orientation.firstAngle, orientation.secondAngle);
            }

            telemetry.addData("Visible Target(s)", robot.stringifyVisibleTargets());

            // move to stone if visible
            VuforiaTrackable trackedStone = robot.getVisibleTrackable("Stone Target");
            if (trackedStone != null) {
                Point3D positionFromSkystone = robot.getPositionFromSkystone();
                Point3D stonePoint3D = new Point3D(trackedStone.getLocation());
                telemetry.addData("Position relative to Skystone", "{X, Y, Z} = %.0f, %.0f, %.0f", positionFromSkystone.x, positionFromSkystone.y, positionFromSkystone.z);
                telemetry.addData("Course", Math.toDegrees(robot.getCourse(positionFromSkystone, stonePoint3D)));
                telemetry.addData("Distance", robot.getDistance(positionFromSkystone, stonePoint3D));
                //robot.simpleMove(robot.getCourse(positionFromSkystone, stonePoint3D), 1, 0, robot.getDistance(positionFromSkystone, stonePoint3D));
            }
        } else {
            telemetry.addData("Visible Target", "None");
        }

        /*
            Example Moving/Turning
            robot.simpleMove(robot.getCourseFromRobot(stonePoint3D), 1, 0, robot.getDistanceFromRobot(stonePoint3D));
            robot.turn(90, 0.5);
            robot.move(robot.getCourseFromRobot(stonePoint3D), 1, new OrientationInfo(145, 0.3), robot.getDistanceFromRobot(stonePoint3D));
         */

        telemetry.update();
        return true;
    }
}
}