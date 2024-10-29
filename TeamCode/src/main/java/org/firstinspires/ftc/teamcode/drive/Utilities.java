package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Utilities", group="Linear Opmode")
public class Utilities extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Constants constants = new Constants(this);
        constants.init();
        constants.hanger.setPower(1);
        telemetry.update();
        int hangerTarget = 0;

        int slideNewPosition = 0;
        int slideTarget = 0;
        int e_tiltTarget = 0;
        constants.hanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        constants.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        constants.hanger.setPower(0);
        constants.collector.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            telemetry.addLine("Gamepad 1 Right Bumper to release hook");
            telemetry.addLine("Gamepad 1 Left Bumper to lower hook");
            telemetry.addLine("Gamepad 1 A to set hook");
            telemetry.addLine("Gamepad 2 Right Bumper lowers hook manually");
            telemetry.addLine("Gamepad 2 Left Bumper to raises hook manually");
            telemetry.addLine("Gamepad 2 Right Trigger to reset Encoders");
            telemetry.addLine("Gamepad 2 x to open claw");
            telemetry.addLine("gamepad 2 y to half open claw");
            telemetry.addLine("gamepad 2 b to close claw");
            telemetry.addLine("gamepad 2 dpad_up to launch drone");
            telemetry.addLine("gamepad 2 dpad_down to reset drone");
            telemetry.addLine("gamepad 2 dpad_left to set the drone");
            telemetry.addLine("I am updating");
            telemetry.addData("Hanger Encoder", hangerTarget);

            telemetry.addData("right stick y",gamepad1.right_stick_y);
            telemetry.addData("left stick y",gamepad1.left_stick_y);

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
            if (gamepad1.left_bumper){
//                constants.hanger.setTargetPosition(15000);//bring down hanger
            }
            if (gamepad1.a) {
                constants.hanger.setTargetPosition(constants.hanger.getCurrentPosition() + 900);//sets hanger
            }
            if (gamepad1.right_stick_y < -0.2){
                constants.rightFront.setPower(1.0);
            }
            else {
                constants.rightFront.setPower(0);

            }
            if (gamepad1.right_stick_y > 0.2){
                constants.rightRear.setPower(1.0);
            }
            else {
                constants.rightRear.setPower(0);

            }
            if (gamepad1.left_stick_y < -0.2){
                constants.leftFront.setPower(1.0);
            }
            else {
                constants.leftFront.setPower(0);
            }
            if (gamepad1.left_stick_y > 0.2){
                constants.leftRear.setPower(1.0);
            }
            else {
                constants.leftRear.setPower(0);
            }

            hangerTarget = constants.hanger.getCurrentPosition();
            if (gamepad2.right_bumper) {
                constants.hanger.setTargetPosition(hangerTarget + 100);
                constants.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.hanger.setPower(1);
            }
            if (gamepad2.left_trigger > 0.5) {

            }

            if (gamepad2.left_bumper) {
                constants.hanger.setTargetPosition(hangerTarget - 100);
                constants.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.hanger.setPower(1);
            }

            if (gamepad2.right_trigger > 0.5) {
                constants.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                constants.hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad2.a) {
            }

            if (gamepad2.b) {
            }

            if (gamepad2.x) {
            }
            if (gamepad2.right_stick_y < -0.2) { //move slide up manually
                if (slideTarget > slideMed) {
                    slideTarget -= 150;
                } else {
                    slideTarget = slideMed;
                }
                constants.slide.setTargetPosition(slideTarget);

            } else if (gamepad2.right_stick_y > 0.2) {
                if (slideTarget < -10) {
                    slideTarget += 150;
                } else {
                    slideTarget = -10;
                }
                constants.slide.setTargetPosition(slideTarget);
            }

            if (gamepad2.y) {
            }

            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) { //tilt slide manually
               constants.hanger.setPower(gamepad2.left_stick_y);
               }
               else {
                   constants.hanger.setPower(0);
                }


//            } else if (gamepad2.left_stick_y > 0.2) {




            //launch drone
            if (gamepad2.dpad_up) {
                constants.collector.setPower(0.5);
            }

            //Reset the drone hammer
            if (gamepad2.dpad_down) {
            }

            //Set the drone
            if (gamepad2.dpad_left) {
            }
            telemetry.update();
        }
    }
}
