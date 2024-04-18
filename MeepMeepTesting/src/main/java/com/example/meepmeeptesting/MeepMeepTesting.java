package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(353.77056000000005), Math.toRadians(353.77056000000005), 13.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 60, Math.toRadians(-90)))
                                // go to purple pixel deposit
                                .splineTo(new Vector2d(-30, 35), Math.toRadians(0))
                                // go to stack
                                .lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(0)))
                                .waitSeconds(1)
                                .forward(15)
                                // go to depositing position
                                .splineTo(new Vector2d(-30.01, 58.96), Math.toRadians(0.00))
                                .lineTo(new Vector2d(24.30, 59.49))
                                .splineTo(new Vector2d(50, 35), Math.toRadians(0.00))
                                .waitSeconds(1.5)
                                // return
                                .back(5)
                                .splineTo(new Vector2d(23.94, 58.96), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-45.20, 51.63), Math.toRadians(200))
                                .splineTo(new Vector2d(-60, 35), Math.toRadians(180.00))
                                .waitSeconds(1.5)
                                // return to deposit
                                .splineTo(new Vector2d(-30.01, 58.96), Math.toRadians(0.00))
                                .lineTo(new Vector2d(24.30, 59.49))
                                .splineTo(new Vector2d(50, 35), Math.toRadians(0.00))
                                .waitSeconds(1.5)
                                // park
                                .strafeLeft(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

