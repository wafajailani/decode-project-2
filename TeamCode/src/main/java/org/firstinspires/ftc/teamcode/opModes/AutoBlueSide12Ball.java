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

//@Autonomous
public class AutoBlueSide12Ball extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-49,-49, Math.toRadians(90)));
        MyActions robot = new MyActions(hardwareMap);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0), // strafe velocity
                new AngularVelConstraint(Math.PI / 2) // rotational velocity
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
        //acceleration, deceleration

        telemetry.addLine("updated code V10 - blue");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.spinUpShooter(),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .setTangent(Math.toRadians(225))
                                        .lineToY(-14)
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.kUp(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(180))
                                .lineToX(-39.5)
                                .stopAndAdd(robot.stopBackIntake())
                                .setTangent(Math.toRadians(270))
                                .lineToY(-20)
                                // .stopAndAdd(robot.stopIntake())
                                .build()
                )
        );





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

  public class MyActions {
        public DcMotorEx fintake, bintake;
        public DcMotorEx st;
        public Servo kicker1, kicker2, kicker3;
        public Limelight3A limelight;
       private static final double SHOOTING_POSITION_KICKER1 = 0.3;
       private static final double SHOOTING_POSITION_KICKER2 = 0.55;
       private static final double SHOOTING_POSITION_KICKER3 = 0.07;

       private static final double INIT_POSITION_KICKER1 = 0.71;
       private static final double INIT_POSITION_KICKER2 = 0.15;
       private static final double INIT_POSITION_KICKER3 = 0.45;

       double limelightHeight  = 14; // cam height from floor in inches
       double targetHeight = 29.5; // // basketHeight - center of the april tag 29.5 - degrees
       double mountAngle = 20.5; // 17.5;//19.0;//  11.7; //27.0;// 17.5;// 25.0;// 32.0; // degrees

       int fiducialId = 0;

        public MyActions(HardwareMap hardwareMap){
            fintake = hardwareMap.get(DcMotorEx.class, "fintake");
            bintake = hardwareMap.get(DcMotorEx.class, "bintake");


            st = hardwareMap.get(DcMotorEx.class, "shooter"); // shooter top motor
            st.setDirection(DcMotorEx.Direction.FORWARD);

            // Set motor to RUN_USING_ENCODER mode for velocity control
            st.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // Set zero power behavior
            st.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


           limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Assume pipeline 0 is AprilTag
            limelight.start();

            // working code
            kicker1 = hardwareMap.get(Servo.class, "kicker1");
            kicker1.setDirection(Servo.Direction.REVERSE);
            kicker1.scaleRange(0.0, 1.0); // full PWM range used
            kicker1.setPosition(0.72); //RESTING_POSITION);


            // working code
            kicker2 = hardwareMap.get(Servo.class, "kicker2");
            kicker2.setDirection(Servo.Direction.FORWARD);
            // ((PwmControl)kicker2).setPwmRange(new PwmControl.PwmRange(500, 2500));
            kicker2.scaleRange(0.0, 1.0); // full PWM range used
            kicker2.setPosition(0.13);


            kicker3 = hardwareMap.get(Servo.class, "kicker3");
            kicker3.setDirection(Servo.Direction.FORWARD);
            kicker3.scaleRange(0.0, 1.0); // full PWM range used
            kicker3.setPosition(0.46);

        }

        public class MyLimeLight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                LLResult result = limelight.getLatestResult();

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    telemetry.addData("ID: ", id);

                    if (id == 21 || id == 22 || id ==23) {
                        telemetry.addLine("seeing an id");
                    }
                }
                telemetry.update();

                return false;
            }
        } 

        public Action limeLightScan() {
            return new MyLimeLight();
        }

      public class KickersUpRandom implements Action {

          @Override
          public boolean run(@NonNull TelemetryPacket telemetryPacket) {

              st.setVelocity(1460);

              sleep(1000);

              kicker1.setPosition(0.31); //0.6); // KICKING_POSITION);
              sleep(250);

              kicker1.setPosition(0.72);

              // --------------------------------------------------

              // --------------------------------------------------
              // ball-2

              sleep(500);
              st.setVelocity(1510); //1490);
              sleep(250);

              kicker2.setPosition(0.53);//0.55 // 0.7
              sleep(250);

              kicker2.setPosition(0.13);//0.3);


              // --------------------------------------------------

              // --------------------------------------------------
              // ball-3

              st.setVelocity(1500); //1700);
              sleep(500);

              kicker3.setPosition(0.07);
              sleep(250);

              kicker3.setPosition(0.46);
              sleep(200);

              return false;
          }
      }

      public Action kUp() {
          return new KickersUpRandom();
      }

       public class BackIntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bintake.setPower(0.7);
                sleep(1100);
                return false;
            }
        }

        public class StopBackIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bintake.setPower(0.0);
                return false;
            }
        }


       public Action BackintakeIn() {
            return new BackIntakeIn();
        }

        public Action stopBackIntake() {
            return new StopBackIntake();
        }

      public class SetShooterOnInit implements Action {

          @Override
          public boolean run(@NonNull TelemetryPacket telemetryPacket) {
              st.setVelocity(1500);
              return false;
          }
      }

      public Action spinUpShooter() {
          return new SetShooterOnInit();
      }

      public class StopShooter implements Action {

          @Override
          public boolean run(@NonNull TelemetryPacket telemetryPacket) {
              st.setVelocity(0);
              return false;
          }
      }

      public Action stopShooter() {
          return new StopShooter();
      }

    }
}
