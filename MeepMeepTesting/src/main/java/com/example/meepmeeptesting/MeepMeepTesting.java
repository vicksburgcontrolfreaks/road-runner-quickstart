package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
//        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        //Provide the initial pose
        Pose2d blueLongStart = new Pose2d(-36, 60, Math.toRadians(270));
        Pose2d blueShortStart = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d redLongStart = new Pose2d(-36, -63, Math.toRadians(90));
        Pose2d redShortStart = new Pose2d(12, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52, 30, Math.toRadians(166), Math.toRadians(45), 14.47)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redShortStart)
                                .forward(9)
                                .lineTo(new Vector2d(12, -53))
                                .lineTo(new Vector2d(-35, -53))
                                .lineTo(new Vector2d(-63, -35))
                                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}