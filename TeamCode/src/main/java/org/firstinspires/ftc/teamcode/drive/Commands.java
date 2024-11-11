/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Commands {
    /* Declare OpMode members. */
    private LinearOpMode controlFreaks;   // gain access to methods in the calling OpMode.
    private Constants constants;

    double driveTurn;
    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double gamepadHypot = 0;
    double gamepadRadians = 0;
    double robotRadians = 0;
    double correctedRobotRadians = 0;
    double movementRadians = 0;
    double gamepadXControl = 0;
    double gamepadYControl = 0;

    public double headingOffset = 0;
    YawPitchRollAngles orientation;

    /* Declare OpMode members. */
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Commands(LinearOpMode opmode, Constants myConstants) {
        controlFreaks = opmode;
        constants = myConstants;
    }

    public void runDuringOp() {
        if (constants.collecting) {
            constants.collector.setPower(1.0);
            constants.c_tilt.setTargetPosition(constants.collectorDown);
//                constants.c_tilt.setPower(constants.c_tiltPower);
//                if (constants.c_tilt.getCurrent(CurrentUnit.MILLIAMPS) > constants.overload) {
//                    constants.c_tilt.setPower(0.0);
        }
        if (constants.collector.getCurrent(CurrentUnit.MILLIAMPS) > constants.collectorOverload) {// we have collected a sample
            constants.collector.setPower(0.0);
            constants.collecting = false;
        }
        if (!constants.collecting && !constants.retracting) {
            constants.c_tilt.setTargetPosition(constants.collectorUp);
            constants.collector.setPower(0);
        }
        int dump = 0;
        int hold = 1;
        if (constants.cll.isPressed()) {
            constants.collectorDown = constants.c_tilt.getCurrentPosition();
        }
        if (constants.delivery && (Math.abs(constants.slide.getCurrentPosition() - constants.currentBasket) < 1000)) {
            constants.bucket.setPosition(dump);
        }
        if (Math.abs(constants.slide.getCurrentPosition() - constants.currentBasket) < 100) {
            constants.collector.setPower(0);
            sleep(800);
            constants.bucket.setPosition(hold);
            sleep(200);
            constants.delivery = false;
            constants.retracting = true;
        }
        if (constants.retracting) {
            constants.c_tilt.setTargetPosition(constants.collectorMed);
            constants.slide.setTargetPosition(constants.slideDown);
            if (Math.abs(constants.slide.getCurrentPosition() - constants.slideDown) < 100) {
                constants.c_tilt.setTargetPosition(constants.collectorUp);
                constants.retracting = false;
            }
        }
    }

    public void fieldOrientedDrive(){
        double left_front_power;
        double right_front_power;
        double left_rear_power;
        double right_rear_power;

        /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
//            angles = constants.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        orientation = constants.imu.getRobotYawPitchRollAngles();
//            float gyro_degrees = (angles.firstAngle) - (float) headingOffset;
        double gyro_degrees = (orientation.getYaw(AngleUnit.DEGREES)) - (float) headingOffset;

        controlFreaks.telemetry.addData("Yaw", ("%.3f"), gyro_degrees);
        controlFreaks.telemetry.addData("hanger", constants.hanger.getCurrentPosition());
        controlFreaks.telemetry.addData("gamepad2.right_stick_y", controlFreaks.gamepad2.right_stick_y);
        controlFreaks.telemetry.addData("gamepad2.left_stick_y", controlFreaks.gamepad2.left_stick_y);
        controlFreaks.telemetry.addData("slidemotor current", constants.slide.getCurrent(CurrentUnit.MILLIAMPS));
        controlFreaks.telemetry.addData("c_tilt pos", constants.c_tilt.getCurrentPosition());
        controlFreaks.telemetry.addData("c_tilt current", constants.c_tilt.getCurrent(CurrentUnit.MILLIAMPS));
        controlFreaks.telemetry.addData("c tilt target", constants.c_tilt.getTargetPosition());
        controlFreaks.telemetry.addData("collector current", constants.collector.getCurrent(CurrentUnit.MILLIAMPS));
        controlFreaks.telemetry.addData("slide pos", constants.slide.getCurrentPosition());
        controlFreaks.telemetry.addData("collecting", constants.collecting);
        controlFreaks.telemetry.addLine("im updating");

        controlFreaks.telemetry.update();

        driveTurn = -controlFreaks.gamepad1.left_stick_x;
        gamepadXCoordinate = -controlFreaks.gamepad1.right_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = controlFreaks.gamepad1.right_stick_y; //this simply gives our y value relative to the driver
        gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

        //finds just how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)

        gamepadRadians = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);// - Math.PI/2; //the inverse tangent of opposite/adjacent gives us our gamepad degree

        robotRadians = (gyro_degrees * Math.PI / 180); //gives us the angle our robot is at, in radians

        movementRadians = gamepadRadians - robotRadians; //adjust the angle we need to move at by finding needed
        // movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(movementRadians) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(movementRadians) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors

        //by multiplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will
        // not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
        right_front_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        right_rear_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        left_front_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        left_rear_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        constants.rightFront.setPower(right_front_power * constants.DRIVE_SPEED);
        constants.leftFront.setPower(left_front_power * constants.DRIVE_SPEED);
        constants.rightRear.setPower(right_rear_power * constants.DRIVE_SPEED);
        constants.leftRear.setPower(left_rear_power * constants.DRIVE_SPEED);
    }

    public void score(int whichBasket) {
        //            set slide position
        constants.slide.setTargetPosition(whichBasket);
//            set delivery to true
        constants.delivery = true;
//            reverse collector
        constants.collector.setPower(-1);
        constants.currentBasket = whichBasket;
//            dump
    }
}