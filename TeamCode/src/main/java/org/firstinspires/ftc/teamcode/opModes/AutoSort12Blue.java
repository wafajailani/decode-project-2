package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoSort12Blue extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-49, -49, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0), // strafe velocity
                new AngularVelConstraint(Math.PI / 2) // rotational velocity
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
        //acceleration, deceleration


        telemetry.addLine("updated code V2 - blue");
        telemetry.update();

        waitForStart();
        Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(startPose)
                                        .setTangent(Math.toRadians(45))
                                        .lineToY(-12)
                                        .build()

                ));

     /*   Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            new SequentialAction(
                            robot.spinUpShooter(),
                            new SleepAction(0.25),
                            robot.stopShooter()
                            ),
                            new SequentialAction(
                            drive.actionBuilder(drive.localizer.getPose())
                            .setTangent(Math.toRadians(225))
                            .lineToY(-18)
                            .build()) //end of sequential action
                    ),
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose())
                                        .stopAndAdd(robot.kUp())
                        )
                )
        )// end of parallel actions
        // end of blocking */
    }

}
