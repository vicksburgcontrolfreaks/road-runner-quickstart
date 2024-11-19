package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Utilities", group="Linear Opmode")
public class Utilities extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Constants constants = new Constants(this);
        constants.init();
        telemetry.update();
        int hangerTarget = 0;

        int slideNewPosition = 0;
        int slideTarget = 0;
        int e_tiltTarget = 0;
        constants.slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        constants.hanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        constants.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        constants.c_tilt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        constants.hanger.setPower(1);
        constants.collector.setPower(0);
        constants.slide.setPower(0);
        constants.c_tilt.setPower(0);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("gamepad2 b sets bucket to right");
            telemetry.addLine("gamepad2 x sets bucket to left");
            telemetry.addLine("gamepad2 leftstick-y controls hanger");
            telemetry.addLine("Gamepad2 rightstick-y controls slide");
            telemetry.addLine("I am updating1");
            telemetry.addData("Hanger Encoder", hangerTarget);
            telemetry.addData("slide pos", constants.slide.getCurrentPosition());
            telemetry.addData("gamepas2 right stick y", gamepad2.right_stick_y);
            telemetry.addData("gamepad2 left stick y", gamepad2.left_stick_y);
            telemetry.addData("right stick y", gamepad1.right_stick_y);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("c tilt pos", constants.c_tilt.getCurrentPosition());
            telemetry.addData("left encoder", constants.leftRear.getCurrentPosition());
            telemetry.addData("right encoder", constants.rightFront.getCurrentPosition());

            telemetry.addData("left front", constants.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("left rear", constants.leftRear.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("right front", constants.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("right rear", constants.rightRear.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("hanger target pos", constants.hanger.getTargetPosition());
            telemetry.addData("hanger current", constants.hanger.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("gamepad2 left trigger ", gamepad2.left_trigger);
            telemetry.addData("gamepad2 right trigger ", gamepad2.right_trigger);

            final int e_tiltPickUp = 0; //The tilt position for picking up a pixel 320 for 5618 and 6494
            final int e_tiltStowed = -475; //The tilt position for moving across the field -30

            final int slidePickup = -250;
            final int slideLow = -1300;
            final int slideMed = -1900;
            final int slideHigh = -2600;
            final int slideTop = -3100;
            boolean tryingToScore = false;

            if (gamepad1.right_bumper) {
//                constants.hanger.setTargetPosition(-1200);//release hanger
            }
            if (gamepad1.left_bumper) {
//                constants.hanger.setTargetPosition(15000);//bring down hanger
            }
            if (gamepad1.a) {
            }
            if (gamepad1.right_stick_y < -0.2) {
                constants.rightFront.setPower(1.0);
            } else {
                constants.rightFront.setPower(0);

            }
            if (gamepad1.right_stick_y > 0.2) {
                constants.rightRear.setPower(1.0);
            } else {
                constants.rightRear.setPower(0);

            }
            if (gamepad1.left_stick_y < -0.2) {
                constants.leftFront.setPower(1.0);
            } else {
                constants.leftFront.setPower(0);
            }
            if (gamepad1.left_stick_y > 0.2) {
                constants.leftRear.setPower(1.0);
            } else {
                constants.leftRear.setPower(0);
            }

            hangerTarget = constants.hanger.getCurrentPosition();

            if (gamepad2.right_bumper) { //score sequence for high basket
//                constants.slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            set slide position
                constants.slide.setTargetPosition(constants.highBasket);
//            set delivery to true
                constants.delivery = true;
//            reverse collector
                constants.collector.setPower(-1);
//            dump
            }
            if (Math.abs(constants.slide.getCurrentPosition() - constants.highBasket) < 100) {
                constants.bucket.setPosition(0);

            }
            if (gamepad2.left_trigger > 0.5) {
                constants.slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            }

            if (gamepad2.left_bumper) {

            }

            if (gamepad2.right_trigger > 0.5) {

            }
            if (gamepad2.a) {
                constants.collector.setPower(1.0);
                constants.c_tilt.setTargetPosition(constants.collectorDown - constants.offset);
                constants.c_tilt.setPower(constants.c_tiltPower);
                if (constants.cll.isPressed()) {
                    constants.offset = constants.c_tilt.getCurrentPosition();
                }
                if (constants.c_tilt.getCurrent(CurrentUnit.MILLIAMPS) > constants.c_tiltOverload) {
                    constants.c_tilt.setPower(0.0);
                }
                if (constants.collector.getCurrent(CurrentUnit.MILLIAMPS) > constants.collectorOverload) {// we have collected a sample
                    constants.collector.setPower(0.0);
                    constants.c_tilt.setTargetPosition(constants.collectorUp);
                    constants.c_tilt.setPower(constants.c_tiltPower);

                }

            }

            if (gamepad2.b) {
                constants.bucket.setPosition(0);
            }

            if (gamepad2.x) {
                constants.bucket.setPosition(1);
            }
            if (gamepad2.right_stick_y < -0.2 || gamepad2.right_stick_y > 0.2) { //move slide up manually
                constants.slide.setPower(gamepad2.right_stick_y);

            } else {
                constants.slide.setPower(0);
            }


            if (gamepad2.y) {
                constants.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.left_stick_y > 0.2) { //tilt slide manually
               constants.hanger.setTargetPosition(constants.hanger.getCurrentPosition() - 100);
               }
            if (gamepad2.left_stick_y < -0.2) { //tilt slide manually
                constants.hanger.setTargetPosition(constants.hanger.getCurrentPosition() + 100);
            }


//            } else if (gamepad2.left_stick_y > 0.2) {




            if (gamepad2.dpad_up) {
                constants.c_tilt.setTargetPosition(constants.collectorUp);
            }

            if (gamepad2.dpad_down) {
                constants.c_tilt.setTargetPosition(constants.collectorDown);

            }

            //Set the drone
            if (gamepad2.dpad_left) {
            }
            telemetry.update();
        }
    }
}
