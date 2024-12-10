/* Driving with mech wheels
 *
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
//    private Commands commands = new Commands(this);
private Commands commands;
    //initalize touch sensor

    Orientation angles;

    private double left_front_power;
    private double right_front_power;
    private double left_rear_power;
    private double right_rear_power;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError = 0;

    private boolean dPadUpIsPressed = false;
    private boolean dPadDownIsPressed = false;
    private boolean dPadLeftIsPressed = false;
    private boolean dPadRightIsPressed = false;
    private int scoreY = 1; //vertical scoring position
    private int scoreX = 1; //horizontal scoring position
    int state = 0;
    int slideTarget = 0;
    int hangerTarget = 0;

    int slideCounter = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

//    private double targetHeading = 0;
//    private double driveSpeed = 0;
//    private double turnSpeed = 0;
//    private double leftSpeed = 0;
//    private double rightSpeed = 0;
//    private final int leftFrontTarget = 0;
//    private final int leftRearTarget = 0;
//    private final int rightFrontTarget = 0;
//    private final int rightRearTarget = 0;


    @Override
    public void runOpMode() {
        constants.init();
        commands = new Commands();
        telemetry.update();
        constants.DRIVE_SPEED = 1;
        constants.TURN_SPEED = 0.75;

        YawPitchRollAngles orientation;

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

        boolean pickingUp = false;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        constants.slide.setTargetPosition(constants.slideUp);

        while (opModeIsActive()) {

            // run until the end of the match (driver presses STOP)

            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
//            angles = constants.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            orientation = constants.imu.getRobotYawPitchRollAngles();
//            float gyro_degrees = (angles.firstAngle) - (float) headingOffset;
            double gyro_degrees = (orientation.getYaw(AngleUnit.DEGREES)) - (float) headingOffset;

//            telemetry.addData("Score X", scoreX);
//            telemetry.addData("Score Y", scoreY);
            telemetry.addData("Yaw", ("%.3f"), gyro_degrees);
//            telemetry.addData("slide_motor", constants.slide_motor.getCurrentPosition());
            telemetry.addData("state", state);
            telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);
            telemetry.addData("slidemotor current", constants.slide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("collector current", constants.collector.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("c_tilt current", constants.c_tilt.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("hanger", constants.hanger.getCurrentPosition());
            telemetry.addData("c_tilt pos", constants.c_tilt.getCurrentPosition());
            telemetry.addData("c tilt target", constants.c_tilt.getTargetPosition());
            telemetry.addData("slide pos", constants.slide.getCurrentPosition());
            telemetry.addData("hanger target", constants.hanger.getTargetPosition());
            telemetry.addData("holdSlideDown", constants.holdSlideDown);
            telemetry.addData("collecting", constants.collecting);
            telemetry.addData("retracting", constants.retracting);
            telemetry.addData("delivery", constants.delivery);
            telemetry.addData("sll", constants.sll);
            telemetry.addLine("im updating");



//            telemetry.addData("green", constants.color.green());
//            telemetry.addData("red", constants.color.red());
//            telemetry.addData("blue", constants.color.blue());
            telemetry.update();

            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = -gamepad1.right_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = gamepad1.right_stick_y; //this simply gives our y value relative to the driver
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

            //Declare other button functions here
            //*****************************     Gamepad 1     **************************************
            //**************   ROBOT SPEED   **************
            if (gamepad1.left_stick_y < -0.2) {
            }

            if (gamepad1.right_bumper) {
                //Slows down robot to half speed
            constants.DRIVE_SPEED = constants.DRIVE_SPEED * 0.5;
            } else {
                constants.DRIVE_SPEED = 1;
            }


            if (gamepad1.left_bumper) { // slow down for precision
                constants.imu.resetYaw();

            }

            if (gamepad1.x) {

            }

            if (gamepad1.a) {

            }

            if (gamepad1.b) {
            }

            //*****************************     Gamepad 2     **************************************
            int dump = 0;
            int hold = 1;
            if (constants.cll.isPressed()) {
                constants.collectorDown = constants.c_tilt.getCurrentPosition();
                constants.collectorUp = constants.collectorDown  - 1080;
//                constants.collectorDown = constants.collectorDown - constants.c_tilt.getCurrentPosition();
            }

            if (constants.sll.isPressed()) {
                constants.slideDown = constants.slide.getCurrentPosition();
                constants.slideUp = constants.slideDown - 1240;
                constants.highBasket = constants.slideDown - 3715;
                constants.lowBasket = constants.slideDown - 1840;
                constants.slide.setTargetPosition(constants.slide.getCurrentPosition());
                constants.holdSlideDown = false;
                constants.slide.setPower(1);
            }
            if  (gamepad2.right_stick_button) {
                while (!constants.sll.isPressed()) {
                    constants.slide.setPower(0.5);
                    constants.holdSlideDown = true;
                    constants.slide.setTargetPosition(constants.slide.getCurrentPosition() + 1000);

                }
            }


            if (gamepad2.left_stick_y > 0.4) {
                //run hanger manually down
                constants.hanger.setTargetPosition(constants.hanger.getCurrentPosition() - 100);
//                constants.hanger.setPower(1);
            }
//            if (gamepad2.left_stick_y < -0.3 && Math.abs(constants.hanger.getCurrentPosition()) > 4000) {
//                constants.hanger.setPower(0);
//            }

                if (gamepad2.left_stick_y < -0.4 && Math.abs(constants.hanger.getCurrentPosition()) < 4000) {
                //run hanger manually up
                constants.hanger.setTargetPosition(constants.hanger.getCurrentPosition() + 100);

            }
//            if (gamepad2.left_stick_y < -0.2 || gamepad2.left_stick_y > 0.2) { //move slide up manually
//                constants.hanger.setPower(-gamepad2.left_stick_y);
//            } else {
//                constants.hanger.setPower(0);
//            }
//             collection sequence
//            start by spinning collector then drop c_tilt when collector current is 400+ stop collector and raise arm
            if (gamepad2.a) {
                //collect
                constants.collecting = true;
            }

            if (constants.collecting) {
                constants.collector.setPower(1.0);
                constants.c_tilt.setTargetPosition(constants.collectorDown);
//                constants.c_tilt.setPower(constants.c_tiltPower);
//                if (constants.c_tilt.getCurrent(CurrentUnit.MILLIAMPS) > constants.overload) {
//                    constants.c_tilt.setPower(0.0);
            }

            if (constants.collector.getCurrent(CurrentUnit.MILLIAMPS) > constants.collectorOverload) {// we have collected a sample
                constants.collector.setPower(0.1);
                constants.c_tilt.setTargetPosition(constants.collectorUp);
                constants.collecting = false;
//                constants.slide.setTargetPosition(constants.slideUp);
                constants.slideMoveDown = true;
            }

            if (!constants.collecting && !constants.retracting) {
                //stops collector
//                constants.c_tilt.setTargetPosition(constants.collectorUp);
                constants.collector.setPower(0);
            }

            if (constants.slideMoveDown && Math.abs(constants.c_tilt.getCurrentPosition() - constants.collectorUp) < 100) {

//                constants.slide.setTargetPosition(constants.slideDown);
                constants.slideMoveDown = false;

            }

            if (gamepad2.y) {
                //puts c_tilt into stowed position
                constants.collecting = false;
                constants.c_tilt.setTargetPosition(constants.collectorUp);
                constants.retracting = false;
            }

            if (gamepad2.x) {
                //sets c_tilt into post hanging position
                constants.collecting = false;
                constants.c_tilt.setTargetPosition(constants.collectorMed);
                constants.retracting = false;
                constants.hanger_tilt.setPosition(0);
                sleep(700);
                constants.hanger_tilt.setPosition(1);

            }

            if (gamepad2.b) {
                //spits out incorrect samples
                constants.collecting = false;
                constants.c_tilt.setTargetPosition(constants.collectorMed);
                constants.collector.setPower(-1);
//                constants.c_tilt.setTargetPosition(constants.collectorUp);
            }

            if (gamepad2.left_trigger > 0.4 ) {
                constants.holdSlideDown = true;
                constants.slide.setTargetPosition(constants.slideDown);
                if (Math.abs(constants.slide.getCurrentPosition() - constants.slideDown) < 100) {
                constants.collector.setPower(-0.5);
                }
            } else {
                constants.holdSlideDown = false;
            }
            if (!constants.collecting && !constants.delivery && !constants.retracting && !constants.holdSlideDown) {
                constants.slide.setTargetPosition(constants.slideUp);
            }

            if (gamepad2.right_stick_y < -0.4) {
                //manually run the c_tilt
                constants.c_tilt.setTargetPosition(constants.c_tilt.getCurrentPosition() + 100);
            }
            if (gamepad2.right_stick_y > 0.4) {
                //manually run the c_tilt
                constants.c_tilt.setTargetPosition(constants.c_tilt.getCurrentPosition() - 100);
            }

                if (gamepad2.right_bumper) {
                    //score sequence for high basket
//                constants.slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                score(constants.highBasket);
            }


            if (gamepad2.left_bumper) {
                //score sequence for low basket
                score(constants.lowBasket);
//                commands.
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
//                constants.c_tilt.setTargetPosition(constants.collectorMed);
                constants.slide.setTargetPosition(constants.slideUp);
                if (Math.abs(constants.slide.getCurrentPosition() - constants.slideUp) < 100) {
                    constants.c_tilt.setTargetPosition(constants.collectorUp);
                    constants.retracting = false;
                }
            }
            if (gamepad2.dpad_down) {
                constants.slide.setTargetPosition(constants.slide.getCurrentPosition() + 40);

            }
            if (gamepad2.dpad_up) {
                constants.slide.setTargetPosition(constants.slide.getCurrentPosition() - 40);


            }
            if (gamepad2.dpad_right) {
                constants.hanger_tilt.setPosition(1);
            }
            if (gamepad2.dpad_left) {
                constants.hanger_tilt.setPosition(0);
            }

//            if (gamepad2.left_stick_y > 0.2) { //
//
//                }
//
//
//            if (gamepad2.dpad_down) {
//                }
//
//

        }
    }
    public void score(int whichBasket){
        //            set slide position
        constants.slide.setTargetPosition(whichBasket);
//            set delivery to true
        constants.delivery = true;
//            reverse collector
        constants.collector.setPower(-0.5);
        constants.currentBasket = whichBasket;
//            dump
    }
}





