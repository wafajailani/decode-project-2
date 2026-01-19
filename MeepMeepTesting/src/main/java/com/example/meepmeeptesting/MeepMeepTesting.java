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
                .setTangent(Math.toRadians(225))
                .lineToY(-8)
                .waitSeconds(2) // shoot presets
                .setTangent(Math.toRadians(180))
                .lineToX(-2)
                .setTangent(Math.toRadians(90))
                .lineToY(-24)
                .setTangent(Math.toRadians(180))
                .lineToX(2)
                .strafeTo(new Vector2d(2, -35)) // or do .setTangent(Math.toRadians(90)) and .lineToY(-35)
          /*      .setTangent(Math.toRadians(90))
                .setTangent(Math.toRadians(45))
                .lineToY(-18)
                .waitSeconds(1) // shoot presets
                .setTangent(Math.toRadians(90))
                .lineToY(-52)
                .lineToY(-49)
                .setTangent(Math.toRadians(180))
                .lineToX(-1)
                .setTangent(Math.toRadians(90))
                .lineToY(-54)
                .lineToY(-45) */

                        /*  .strafeToLinearHeading(new Vector2d(5, -60), Math.toRadians(Math.PI * 0.6))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(-44) */
                //    .strafeToLinearHeading(new Vector2d(7, -59), - 0.7 * Math.PI)
                // .lineToY(-20)
                // .waitSeconds(3) // shoot row 1

                //   .strafeToLinearHeading(new Vector2d(4, -62), Math.PI * 1.2)
                // .setTangent(Math.toRadians(150))
                //    .lineToX(12)

                //    .setTangent(Math.toRadians(90))
                //     .lineToY(-52)
                //    .setTangent(Math.toRadians(-45))
                //   .strafeTo(new Vector2d(-20, -27)) // 20 // 23
                //   .lineToX(-35)
                //     .waitSeconds(3) //shoot row 2
                //                .strafeToLinearHeading(new Vector2d(0, -58), -Math.PI )
                //// .setTangent(Math.toRadians(150))
                ///.lineToX(12)

                //    .turnTo(Math.toRadians(45))

                /* //12 BALL PATH  - last three artifacts overflow, classifier full at end of auto
                                 .setTangent(Math.toRadians(90))
                                 .setTangent(Math.toRadians(45))
                                 .lineToY(-18)
                                 .waitSeconds(3) // shoot presets
                                 .setTangent(Math.toRadians(90))
                                 .lineToY(-52)
                                 .lineToY(-20)
                                 .waitSeconds(3) // shoot row 1
                                 .setTangent(Math.toRadians(150))

                             //    .turnTo(Math.toRadians(45))
                                 .lineToX(12)
                              //   .turn(Math.toRadians(-5.0))

                                 .setTangent(Math.toRadians(90))
                                 .lineToY(-52)
                                 .setTangent(Math.toRadians(-45))
                                 .strafeTo(new Vector2d(-20, -27)) // 20 // 23
                              //   .lineToX(-35)
                                 .waitSeconds(3) //shoot row 2
                                 .setTangent(Math.toRadians(-12))
                                 .lineToX(28) //36
                                 .setTangent(Math.toRadians(90))
                                 .lineToY(-52)
                                 .setTangent(Math.toRadians(-30))
                                 .strafeTo(new Vector2d(-24, -27))
                                 //.lineToX(-34) // shoot row 3
                                 .setTangent(180)
                                 .strafeTo(new Vector2d(34, -36))
                                //TODO: strafeTo either far zone or near side and shoot - find which is quicker*/
                /* 12 BALL PATH - shoot 3, open gate and collect and shoot row 2, row 1, and row 3
                 * PREDICTED TIME: 21.90 SECONDS */
                /*                    .setTangent(Math.toRadians(90))
                                  .setTangent(Math.toRadians(45))
                                  .lineToY(-8)
                                  //.lineToXLinearHeading(-16, -Math.toRadians(90))
                  // .lineToXConstantHeading(24, baseVelConstraint, baseAccelConstraint)
                                  .waitSeconds(3) // shoot presets
                                  .strafeTo(new Vector2d(-11, -50)) */
                /*      .setTangent(Math.toRadians(90))
                      .lineToY(-60)
                      .lineToY(-30)
                      .setTangent(Math.toRadians(180))
                      .lineToX(-12)
                      .setTangent(Math.toRadians(90))
                      .lineToY(-52) */
                //    .turn(Math.toRadians(-90.0))





                /*         .lineToY(-20)
                              .waitSeconds(3) // shoot row 1
                              .setTangent(Math.toRadians(150))
                              .lineToX(8)
                              .setTangent(Math.toRadians(90))
                              .lineToY(-52)
                              .setTangent(Math.toRadians(-25))
                              .strafeTo(new Vector2d(-12, -19)) // 20 // 23 */
                // (-10.5, -28.5)
                /* .waitSeconds(3) // shoot row 2
                 .setTangent(Math.toRadians(-12))
                 .lineToX(28) //36
                 .setTangent(Math.toRadians(90))
                 .lineToY(-52)
                 .setTangent(Math.toRadians(-30))
                 .strafeTo(new Vector2d(-12, -19)) */
                //shoot row 3
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
