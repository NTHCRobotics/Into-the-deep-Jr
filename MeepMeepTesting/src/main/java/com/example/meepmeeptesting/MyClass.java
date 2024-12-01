package com.example.meepmeeptesting;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.SleepAction;


public class MyClass{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12.8, 62.7, 90))
                .setReversed(true)
                . splineTo(new Vector2d(55, 58), Math.toRadians(40))
                        .setReversed(false)
                               // Sample 1
                                .splineTo(new Vector2d(47,34),Math.toDegrees(40))
                                .setReversed(true)
                                .splineTo(new Vector2d(55,58),Math.toRadians(40))
                        //Sample 2
                        .setReversed(false)
                                .splineTo(new Vector2d(57,34),Math.toDegrees(40))
                                .setReversed(true)
                                .splineTo(new Vector2d(55,58),Math.toRadians(40))
                        // Sample 3
                        .setReversed(false)
                                .splineTo(new Vector2d(57,34),Math.toRadians(330))
                                .setReversed(true)

                //.setReversed(true)
                //  .splineTo(new Vector2d(-19, -10.4), Math.toRadians(225))











                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}