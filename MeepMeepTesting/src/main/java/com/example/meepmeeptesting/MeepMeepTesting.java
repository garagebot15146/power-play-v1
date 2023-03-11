package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double toPoleBack = 48;
    public static double toPoleLineX = 32.5;
    public static double toPoleLineY = -4.5;
    public static double toPoleLineH = -16;

    public static double parkCenterLineX = 33;
    public static double parkCenterLineY = -16.5;
    public static double parkCenterLineH = 0;

    public static double parkLeftMove = 23;
    public static double parkLeftTurn = 90;

    public static double parkRightMove = 23;
    public static double parkRightTurn = 90;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34, -72 + (15.5 / 2), Math.toRadians(270)))
                                .back(toPoleBack)
                                .lineToLinearHeading(new Pose2d(toPoleLineX, toPoleLineY, Math.toRadians(toPoleLineH)))
                                .lineToLinearHeading(new Pose2d(35, -25, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}