package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(353.77056000000005), Math.toRadians(353.77056000000005), 13.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -63.42, Math.toRadians(90)))
                                .splineTo(new Vector2d(-31.5, -39.5), Math.toRadians(50.00))
                                .back(13)
                                .lineToLinearHeading(new Pose2d(-55, -36.1, Math.toRadians(0.00)))
                                

  /*                              .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                
                                
                                .waitSeconds(2)
                                .strafeLeft(24.35)
                                .back(7.055)
                                .forward(110)
                                //Put arm up during this period of time
                                .strafeRight(32)
                                .waitSeconds(2)
                                .strafeLeft(32)
                                .back(110)
                                .waitSeconds(2)
                                .forward(114.23)
                                .waitSeconds(2)
                                .back(10)
 */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

