
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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(270)))
                        .setTangent(Math.toRadians(45))
                        .lineToY(-20)
                        .build());

                // near blue with strafe 24.15s

/*  myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(270)))//new Pose2d(-48, -52, Math.toRadians(90)))
                                .setTangent(Math.toRadians(45))
                                .lineToY(-14)
                                .waitSeconds(2) // shoot preset artifacts and scan motif pattern
                                .lineToY(-45) // intake row 1
                                .setTangent(Math.toRadians(180))
                                .strafeTo(new Vector2d(0, -55)) // open gate
                              //  .waitSeconds(1) // hold open for 1 sec
                                .strafeTo(new Vector2d(-22, -22))
                                .waitSeconds(2) // shoot row 1
                                .strafeTo(new Vector2d(12, -34))
                                .strafeTo(new Vector2d(12, -45)) // intake row 2
                                .strafeTo(new Vector2d(-22, -22))
                                .waitSeconds(2) // shoot row 2
                                .strafeTo(new Vector2d(34, -34))
                                .strafeTo(new Vector2d(34, -45)) // intake row 3
                                .strafeTo(new Vector2d(-22, -22))
                                .waitSeconds(2) // shoot row 3
                                .strafeTo(new Vector2d(0, -45))
                                // go to gate and don't open it *//*



                            */
/* //near blue with spline 22.75s
                            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(270)))//new Pose2d(-48, -52, Math.toRadians(90)))
                                .setTangent(Math.toRadians(90))
                                .lineToY(-8)
                                .waitSeconds(2) // shoot preset artifacts and scan motif pattern
                                .strafeTo(new Vector2d(-12, -34))
                                .setTangent(Math.toRadians(90))
                                .lineToY(-44)
                                .strafeTo(new Vector2d(0, -59))
                                .waitSeconds(0.5) //open gate
                                .strafeTo(new Vector2d(-24, -24))
                                .waitSeconds(2) // shoot row 1
                                .splineToConstantHeading(new Vector2d(11, -28), - (Math.PI /2))
                                .splineToConstantHeading(new Vector2d(11, -44), (Math.PI/ 2))
                                .splineToConstantHeading(new Vector2d(-24 ,-24), (Math.PI / 2))
                                .waitSeconds(2) // shoot row 2
                                .splineToConstantHeading(new Vector2d(34 , -40), (-Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(34, -49), (Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(-24, -20), (Math.PI / 2))
                                .waitSeconds(2) // shoot row 3
                                .strafeTo(new Vector2d(0, -47)) // go to gate *//*



        */
/* // near red with spline 22.12s
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 49, Math.toRadians(90)))//new Pose2d(-48, -52, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-12, 20))
                                .waitSeconds(2) // shoot preset artifacts and scan motif pattern
                                .setTangent(Math.toRadians(90))
                                .lineToY(45) // intake row
                                .strafeTo(new Vector2d(0, 65))
                                .lineToY(20)
                                .waitSeconds(2) // shoot artifacts
                                .splineToConstantHeading(new Vector2d(11, 39), (Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(11, 45), (Math.PI / 2)) // intake row
                                .splineToConstantHeading(new Vector2d(-20, 15), (Math.PI / 2))
                                .waitSeconds(2) // shoot row
                                .splineToConstantHeading(new Vector2d(34, 33), (Math.PI / 2))
                                .splineToConstantHeading(new Vector2d(34, 45), (Math.PI / 2)) // intake row
                                .strafeTo(new Vector2d(-14, 15))
                                .waitSeconds(2) // shoot artifacts
                                .strafeTo(new Vector2d(0, 46)) // go to gate and don't open it *//*


      */
/*  // near red with strafe 21.64s
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 49, Math.toRadians(90)))//new Pose2d(-48, -52, Math.toRadians(90)))
                .strafeTo(new Vector2d(-12, 20))
                .waitSeconds(2) // shoot presets and scan motif pattern
                .setTangent(Math.toRadians(90))
                .lineToY(45) // intake row 1
                .strafeTo(new Vector2d(0, 62))
                .lineToY(22)
                .waitSeconds(2) // shoot row 1
                .setTangent(Math.toRadians(90))
                .strafeTo(new Vector2d(12, 32))
                .strafeTo(new Vector2d(12, 45)) // intake row 2
                .strafeTo(new Vector2d(-22, 22))
                .waitSeconds(2) // shoot row 2
                .strafeTo(new Vector2d(34, 32))
                .strafeTo(new Vector2d(34, 45)) // intake row 3
                .strafeTo(new Vector2d(-22, 22))
                .waitSeconds(2) // shoot row 3
                .strafeTo(new Vector2d(0, 55)) // go to gate don't open  *//*


             */
/*   // far blue strafe only 9 artifacts 15.58s
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(57, -47, Math.toRadians(270)))
                                .setTangent(Math.toRadians(90))
                                .lineToY(-20)
                                .waitSeconds(2) // scan motif pattern and shoot presets
                                .setTangent(Math.toRadians(180))
                                .lineToX(36)
                                .setTangent(Math.toRadians(90))
                                .lineToY(-52) // intake row 3
                                .strafeTo(new Vector2d(59, -19))
                                .waitSeconds(2) // shoot row 3
                                .strafeTo(new Vector2d(10, -32))
                                .strafeTo(new Vector2d(10, -45)) // intake row 2
                                .strafeTo(new Vector2d(59, -19))
                                .waitSeconds(2) // shoot row 3 *//*


           */
/*     // far red strafe only 9 artifacts 15.58s
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(57, 47, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(90))
                                        .lineToY(20)
                                        .strafeTo(new Vector2d(35, 33))
                                        .strafeTo(new Vector2d(35, 43)) // intake row 3
                                        .strafeTo(new Vector2d(60, 17))
                                        .waitSeconds(2) // shoot row 3
                        .strafeTo(new Vector2d(10, 32))
                        .strafeTo(new Vector2d(10, 45)) // intake row 2
                        .strafeTo(new Vector2d(59, 19)) *//*





                                //.strafeTo(new Vector2d(0,0))

*/

             //   .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}