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

package org.firstinspires.ftc.opmodes.auto;

import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.state.Course;
import org.firstinspires.ftc.robotlib.state.ServoState;

class SkystoneDeliverAutonomous extends AutonomousBase {
    @Override
    public void initializeOpmode() {
        robot.trackables.activate();
    }

    @Override
    public void startOpmode() {
        robot.move(Course.BACKWARD, 0.7, null, 10);
        robot.move(robot.correctMovement(Course.RIGHT), 0.5, null, 34);

        for (int i = 0; i < 1; i++) {
            //if (i > 0) robot.turn(180);

            robot.scanWait(2);
            if (robot.isTrackableVisible() && robot.isSkystoneVisible()) {
                // Get Skystone
                Point3D positionFromSkystone = robot.getPositionFromSkystone();
                Point3D stonePoint3D = new Point3D(robot.getTrackedSkystone().getLocation());
                telemetry.addData("Position relative to Skystone", "{X, Y, Z} = %.0f, %.0f, %.0f", positionFromSkystone.x, positionFromSkystone.y, positionFromSkystone.z);
                robot.move(robot.getCourse(positionFromSkystone, stonePoint3D), 0.3, null, robot.getDistance(positionFromSkystone, stonePoint3D) - 2);
            } else robot.move(Course.BACKWARD, 0.5, null, 5);

            robot.turn(90);

            // Intake Stone
            robot.turn(90);
            robot.hardware.updateDeliveryStates(ServoState.FLOOR);
            robot.hardware.blockGrabber.setPosition(ServoState.OPEN);
            robot.wait(2.0);

            robot.hardware.blockGrabber.setPosition(ServoState.CLOSED);
            robot.wait(0.2);
            robot.hardware.updateDeliveryStates(ServoState.ONEBLOCKHOVER);
            robot.wait(1.0);
            robot.move(Course.BACKWARD, 0.5, null, 8);

            // Deliver Stone
            robot.move(robot.correctMovement(Course.LEFT), 0.5, null, 71);
            robot.move(Course.FORWARD, 0.5, null, 14);
            if (i == 0) robot.hardware.updateDeliveryStates(ServoState.ONEBLOCKDEPOSIT);
            else robot.hardware.updateDeliveryStates(ServoState.TWOBLOCKDEPOSIT);
            robot.hardware.blockGrabber.setPosition(ServoState.OPEN);
            robot.wait(2.0);

            // Move to the loading zone
            robot.move(Course.BACKWARD, 0.5, null, 14);
            robot.move(robot.correctMovement(Course.RIGHT), 0.5, null, 94);
        }

        robot.move(robot.correctMovement(90), 0.5, null, 46);
    }

    @Override
    public void endOpmode() {
        robot.trackables.deactivate();
    }
}