package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Vector;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setDimensions(16, 16)
                .build();

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(70.0), // strafe velocity
                new AngularVelConstraint(Math.PI / 2) // rotational velocity
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-5.0, 25.0);
        //                                                              acceleration, velocity


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(270)))//new Pose2d(-48, -52, Math.toRadians(90)))
                                .setTangent(Math.toRadians(90))
                                .lineToY(-8)
                                .waitSeconds(1) // limelight scans april tag
                                .strafeTo(new Vector2d(-12, -34))
                                .setTangent(Math.toRadians(90))
                                .lineToY(-44)
                                .strafeTo(new Vector2d(0, -59))
                                .waitSeconds(0.5) //hold open the gate
                                .strafeTo(new Vector2d(-24, -24))

                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(11, -28), - (Math.PI /2))
                                .splineToConstantHeading(new Vector2d(11, -44), (Math.PI/ 2))
                                .splineToConstantHeading(new Vector2d(-24 ,-24), (Math.PI / 2))
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(34 , -40), (-Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(34, -49), (Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(-24, -20), (Math.PI / 2))
                                //.splineToConstantHeading(new Vector2d(34, -40), - (Math.PI / 2))
                /*.setTangent(Math.toRadians(225))
                .lineToY(-8)
                .waitSeconds(2) // shoot presets
                .setTangent(Math.toRadians(180))
                .lineToX(-2)
                .setTangent(Math.toRadians(90))
                .lineToY(-24)
                .setTangent(Math.toRadians(180))
                .lineToX(2)
                .strafeTo(new Vector2d(2, -35)) */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
