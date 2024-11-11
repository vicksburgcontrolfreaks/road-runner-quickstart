/* Driving with mech wheels
 *
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*********************************************/

@TeleOp(name = "TeleOpFieldOriented", group = "Linear Opmode")
//@Disabled

public class TeleOpFieldOriented extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.drive.Utilities Utilities;

    Constants constants = new Constants(this);
    Commands commands = new Commands(this, constants);

    //    private Commands commands = new Commands(this);
    //initalize touch sensors

    @Override
    public void runOpMode() {

        constants.init();
        telemetry.update();
        constants.DRIVE_SPEED = 0.75;
        constants.TURN_SPEED = 0.50;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            // run until the end of the match (driver presses STOP)
            commands.runDuringOp();
            commands.fieldOrientedDrive();

            //Declare other button functions here
            //*****************************     Gamepad 1     **************************************
            //**************   ROBOT SPEED   **************
            if (gamepad1.left_stick_y < -0.2) {

            }

            if (gamepad1.left_bumper) { // slow down for precision

            }

            if (gamepad1.x) {

            }

            if (gamepad1.a) {
            }

            if (gamepad1.b) {
            }

            //*****************************     Gamepad 2     **************************************

             // collect sample
            if (gamepad2.a) {
                constants.collecting = true;
            }
            //reject collected piece
            if (gamepad2.y) {
                constants.collecting = false;
                constants.collector.setPower(-1); //you have to hold button until piece is ejected
            }

            if (gamepad2.x) {

            }

            if (gamepad2.b) {

            }

            if (gamepad2.left_trigger > 0.5) {

            }

            if (gamepad2.right_stick_y < -0.2) { //

            }
            if (gamepad2.right_bumper) { //score sequence for high basket
                commands.score(constants.highBasket);
            }


            if (gamepad2.left_bumper) {
                commands.score(constants.lowBasket);
            }

            if (gamepad2.left_stick_y > 0.2) {

                }

            if (gamepad2.dpad_up) {
                }

            if (gamepad2.dpad_down) {
                }

        }// end of while (OpModeIsActive)
    }// end of runOpMode
}// end of class





