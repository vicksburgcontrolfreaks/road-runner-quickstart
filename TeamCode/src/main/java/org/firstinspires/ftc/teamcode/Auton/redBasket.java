/*
https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
 */

package org.firstinspires.ftc.teamcode.Auton;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Commands;
import org.firstinspires.ftc.teamcode.drive.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Red Basket", group = "Autonomous")
public class redBasket extends LinearOpMode {
    Constants constants = new Constants(this);
    Commands commands = new Commands(this, constants);
//    public class Slide {
//        private DcMotorEx slide;
//
//        public Slide(HardwareMap hardwareMap) {
//            slide = hardwareMap.get(DcMotorEx.class, "slide");
//            slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            slide.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class SlideUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    slide.setPower(0.8);
//                    initialized = true;
//                }
//
//                double pos = slide.getCurrentPosition();
//                packet.put("SlidePos", pos);
//                if (pos < 3000.0) {
//                    return true;
//                } else {
//                    slide.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action SlideUp() {
//            return new SlideUp();
//        }
//
//        public class SlideDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    slide.setPower(-0.8);
//                    initialized = true;
//                }
//
//                double pos = slide.getCurrentPosition();
//                packet.put("SlidePos", pos);
//                if (pos > 100.0) {
//                    return true;
//                } else {
//                    slide.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action slideDown(){
//            return new SlideDown();
//        }
//    }

//    public class Bucket {
//        private Servo bucket;
//
//        public Bucket(HardwareMap hardwareMap) {
//            bucket = hardwareMap.get(Servo.class, "bucket");
//        }
//
//        public class Dump implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                bucket.setPosition(0.0);
//                return false;
//            }
//        }
//        public Action dump() {
//            return new Dump();
//        }
//
//        public class Hold implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                bucket.setPosition(1.0);
//                return false;
//            }
//        }
//        public Action hold() {
//            return new Hold();
//        }
//    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        Bucket bucket = new Bucket(hardwareMap);
//        Slide slide = new Slide(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a bucket tightening.
//        Actions.runBlocking(bucket.dump());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
//                        slide.SlideUp(),
//                        bucket.hold(),
//                        slide.slideDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}