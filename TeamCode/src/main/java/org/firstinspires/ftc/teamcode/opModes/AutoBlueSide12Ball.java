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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoBlueSide12Ball extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-49,-49, Math.toRadians(270)));
      //  Intake intake = new Intake(hardwareMap);


        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0), // strafe velocity
                new AngularVelConstraint(Math.PI / 2) // rotational velocity
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
        //acceleration, deceleration

        Pose2d currPose = drive.localizer.getPose();

        telemetry.addLine("updated code V5 - blue");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(Math.toRadians(225))
                        .lineToY(-8)
                        .setTangent(Math.toRadians(180))
                        .lineToX(-2)
                        .setTangent(Math.toRadians(90))
                        .lineToY(-24)
                        .lineToY(-4)
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(180))
                        .lineToX(22)
                        .setTangent(Math.toRadians(90))
                        .lineToY(-24)
                        .strafeTo(new Vector2d(-8, -4), baseVelConstraint)
                        //.lineToY(-6, baseVelConstraint)
                /*        .setTangent(Math.toRadians(90))
                        .stopAndAdd(intake.intakeIn())
                        .lineToY(-16)
                        .stopAndAdd(intake.stopIntake())
                        .lineToY(0.0) */





                        .build());



    }

 /*   public class Intake {

        public DcMotorEx intake;

        public Intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        }

        public class IntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }

        public class StopIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setVelocity(0.0);
                return false;
            }
        }
        public Action intakeIn() {
            return new IntakeIn();
        }

        public Action stopIntake() {
            return new StopIntake();
        }

    } */
}
