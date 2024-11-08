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

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

public class Constants {
    /*
     * These are motor constants that should be listed online for your motors.
     */

    public static final double TICKS_PER_REV = 537.6; //537.6
    public static final double MAX_RPM = 312.5; //

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 0.05208; // output (wheel) speed / input (motor) speed 1/19.2
    public static double TRACK_WIDTH = 14.35; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
//    public static double kV = 0.016 / rpmToVelocity(MAX_RPM);
    public static double kV = 0.0165;
    public static double kA = 0.0015;
    public static double kStatic = 0.035;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 52; //in/s
    public static double MAX_ACCEL = 30; //in/s^2
    public static double MAX_ANG_VEL = Math.toRadians(215); //degrees/s
    public static double MAX_ANG_ACCEL = Math.toRadians(45); //degrees/s^2

    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    /* Declare OpMode members. */
    public LinearOpMode controlFreaks;   // gain access to methods in the calling OpMode.
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Constants(TeleOpFieldOriented opmode) {controlFreaks = opmode;}
    public Constants(Utilities utilities) {controlFreaks = utilities;}
//    public Constants() {controlFreaks = utilities;}

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx slide = null;
    DcMotorEx leftFront               = null;
    DcMotorEx rightFront              = null;
    DcMotorEx rightRear               = null;
    DcMotorEx leftRear                = null;
    DcMotorEx hanger                  = null;
    DcMotorEx collector               = null;
    DcMotorEx c_tilt                  = null;
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    public IMU imu;
    // Define a variable for our color sensor
//    ColorSensor color;
        // Get the color sensor from hardwareMap

//    // Initialize Touch Sensors
    TouchSensor sll;     // Touch sensor for slide lower limit CH 0-(1)
    TouchSensor cll;     // Touch sensor for collector lower limit CH 2-(3)

//Initialize Servos

    public Servo bucket = null;
    Servo hanger_tilt = null;

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading       = 0;
    private double  driveSpeed          = 0;
    private double  turnSpeed           = 0;
    private double  leftSpeed           = 0;
    private double  rightSpeed          = 0;
    private int     leftFrontTarget     = 0;
    private int     leftRearTarget      = 0;
    private int     rightFrontTarget    = 0;
    private int     rightRearTarget     = 0;
    public boolean         delivery            = false;
    boolean         retracting          = false;
    boolean         collecting          = false;
    public int             currentBasket       = 0;
    int             counter             = 0;
    int             collectorDown       = 450;
    int             collectorMed        = 120;
    int             collectorUp         = 20;
    int             slideDown           = -100;
    int             highBasket          = -3750;
    int             lowBasket           = -1950;
    int             highBar             = 1000;
    int             lowBar              = 250;
    int             offset              = 200;
    boolean         START_LEFT;
    double          TURN_SPEED;
    double          DRIVE_SPEED;
    double          collectorOverload   = 450;
    double          c_tiltOverload      = 50000;

    double          c_tiltPower         = 0.1;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;     // REV HD Hex
    static final double     DRIVE_GEAR_REDUCTION    = 15.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    static final double FEET_PER_METER = 3.28084;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Initialize Motors (note: need to use reference to actual OpMode).
        leftFront   = controlFreaks.hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront  = controlFreaks.hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear    = controlFreaks.hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear   = controlFreaks.hardwareMap.get(DcMotorEx.class, "rightRear");
        slide       = controlFreaks.hardwareMap.get(DcMotorEx.class, "slide");
        hanger      = controlFreaks.hardwareMap.get(DcMotorEx.class,"hanger");
        collector   = controlFreaks.hardwareMap.get(DcMotorEx.class, "collector");
        c_tilt      = controlFreaks.hardwareMap.get(DcMotorEx.class, "c_tilt");
        bucket      = controlFreaks.hardwareMap.get(Servo.class, "bucket");
        cll         = controlFreaks.hardwareMap.get(TouchSensor.class, "cll");
        sll         = controlFreaks.hardwareMap.get(TouchSensor.class, "sll");


//        color = controlFreaks.hardwareMap.get(ColorSensor.class, "Color");

        hanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setTargetPosition(0);
        hanger.setPower(0.5);
        hanger.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hanger.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        c_tilt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        c_tilt.setTargetPosition(0);
        c_tilt.setPower(0.5);
        c_tilt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        collector.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setPower(1);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set motor directions
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);


        // Initialize Servos



                  // define initialization values for IMU, and then initialize it.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //TODO:
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu = controlFreaks.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu = controlFreaks.hardwareMap.get(BHI260IMU.class, "imu");
//        imu.initialize(parameters);
        controlFreaks.telemetry.addLine("init complete");
        controlFreaks.telemetry.update();
    }


//    public void runAprilTag(){
//        int cameraMonitorViewId = controlFreaks.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", controlFreaks.hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(controlFreaks.hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
//            } //640x480, 1280x720, 1024x768, 800x448, 960x720, 960x544, 864x480, 848x480, 800x600, 800x448, 640x360, 352x288, 320x240, 1920x1080
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        controlFreaks.telemetry.setMsTransmissionInterval(50);
//
//
//        while (!controlFreaks.isStarted() && !controlFreaks.isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    controlFreaks.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    controlFreaks.telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        controlFreaks.telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        controlFreaks.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                controlFreaks.telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    controlFreaks.telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    controlFreaks.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            controlFreaks.telemetry.update();
//            controlFreaks.sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        camera.closeCameraDevice(); //shut off the camera to preserve battery
//
//        if(tagOfInterest != null)
//        {
//            controlFreaks.telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            controlFreaks.telemetry.update();
//        }
//        else
//        {
//            controlFreaks.telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            controlFreaks.telemetry.update();
//        }
//    }

    public void runWithEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void floatChassis(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakeChassis(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            controlFreaks.telemetry.addData("Motion", "Drive Straight");
            controlFreaks.telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            controlFreaks.telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());
        } else {
            controlFreaks.telemetry.addData("Motion", "Turning");
        }

        controlFreaks.telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        controlFreaks.telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        controlFreaks.telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        controlFreaks.telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
//        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        controlFreaks.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        controlFreaks.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        controlFreaks.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        controlFreaks.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        controlFreaks.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.x)));
        controlFreaks.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.y)));
        controlFreaks.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.z)));
    }
}
