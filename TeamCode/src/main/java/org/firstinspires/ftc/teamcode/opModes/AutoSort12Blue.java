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
        Shoot shooter = new Shoot(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0), // strafe velocity
                new AngularVelConstraint(Math.PI / 2) // rotational velocity
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
        //acceleration, deceleration


        telemetry.addLine("updated code V5 - blue");
        telemetry.update();

        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        shooter.autoResetLimelightFocus(),
                        new SequentialAction(
                                drive.actionBuilder(startPose)
                                        .setTangent(Math.toRadians(45))
                                        .stopAndAdd(shooter.spinUpShooter())
                                        .lineToY(-10)
                                        .stopAndAdd(shooter.kickUpInitial())
                                        .waitSeconds(1.2)
                                        .stopAndAdd(intake.startFIntake())
                                        .lineToY(-55)
                                        .lineToY(-35)

                                        .build())

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

    public class Shoot {

        private DcMotorEx st, turret;
        private Servo kicker1, kicker2, kicker3;
        private Limelight3A limelight;

        private DcMotorEx fintake, bintake;
      //  private BNO055IMU imu;

        // Define angular positions (0.0 to 1.0 maps to your servo's range)
        // For 270 degree servo: 0.0 = 0°, 0.5 = 135°, 1.0 = 270°
        private static final double RESTING_POSITION = 0.13;
        private static final double MAX_POSITION = 1.0;
        private static final double KICKING_POSITION = 0.5; // Center/135°

        private static final double GEAR_RATIO = 196.0 / 52.0;
        private static final double MOTOR_PPR = 384.0; // Pulses per revolution
        private static final double TURRET_PPR = MOTOR_PPR * GEAR_RATIO; // Pulses per turret revolution

        // Angle limits
        private static final double MAX_ANGLE_DEGREES = 30.0;
        private static final double MIN_ANGLE_DEGREES = -30.0;

        // Convert degrees to encoder ticks
        private static final double TICKS_PER_DEGREE = TURRET_PPR / 360.0;

        // game
        double limelightHeight  = 14; // cam height from floor in inches
        double targetHeight = 29.5; // // basketHeight - center of the april tag 29.5 - degrees
        double mountAngle = 17.5;//19.0;//  11.7; //27.0;// 17.5;// 25.0;// 32.0; // degrees

        // ----------------------------
        // Swyft Motor Encoder Math
        // ----------------------------
        // 28 CPR internal encoder * 12.7 gear ratio
        static final double TICKS_PER_REV = 28 * 12.7; // = 356

        static final double WHEEL_DIAMETER_INCHES = 3.78; // Swyft wheel
        static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

        static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

        // Straight correction
        static final double HEADING_K = 0.02; // increase if drift

        // open close the stopper
        double CLOSE_STOPPER = 115.0/300.0;// 105.0 // 70.0 block the balls
        double OPEN_STOPPER  = 55.0/300.0; // 39 allow the balls to the shooter

        // intake velocity calculation
        double MOTOR_MAX_RPM_INTAKE = 435.0;
        double ENCODER_PPR_INTAKE = 384.0;
        double GEAR_RATIO_INTAKE =  13.7;
        double MAX_TICKS_PER_SECOND_INTAKE = (MOTOR_MAX_RPM_INTAKE * ENCODER_PPR_INTAKE * GEAR_RATIO_INTAKE) / 60.0;

        double totalRevIntake = ENCODER_PPR_INTAKE * GEAR_RATIO_INTAKE; // 5260.8
        double rpmIntake = MOTOR_MAX_RPM_INTAKE / GEAR_RATIO_INTAKE; // 31.75 rpm
        double rpsIntake = rpmIntake / 60.0;
        double veloIntake = totalRevIntake * rpsIntake;

        int step = 0;

        double maxTurretDegrees = 30.0;
        double minTurretDegrees = -30.0;
        double turretGearTeeth = 196.0;
        double pinionGearTeeth = 52.0;
        double gearRatio = turretGearTeeth / pinionGearTeeth; // ~3.769
        double encoderTicksPerRevMotor = 384.0;
        double encoderTicksPerTurretRev = encoderTicksPerRevMotor * gearRatio;

        double degreesPerTick = 360.0 / encoderTicksPerTurretRev;

        private int BLUE_ALLIANCE = 20;
      //  private boolean turretAutoFocus = true;
      private boolean isTurretInAutoFocus = true;

        public Shoot(HardwareMap hardwareMap) {

            st = hardwareMap.get(DcMotorEx.class, "shooter"); // shooter top motor
            st.setDirection(DcMotorEx.Direction.FORWARD);

            // Set motor to RUN_USING_ENCODER mode for velocity control
            st.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            // Set zero power behavior
            st.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

          /*  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

*/
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setDirection(DcMotorEx.Direction.REVERSE);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Assume pipeline 0 is AprilTag
            limelight.start();

        }

        public class AutoResetLimelight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                limeLightAutoFocus();
                return true;
            }
        }

            private void limeLightAutoFocus() {

                if(!isTurretInAutoFocus) {
                    return;
                }

                int fiducialId = 0;
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> fidResult = result.getFiducialResults();
                    fiducialId = 0;
                    for (LLResultTypes.FiducialResult fr : fidResult) {
                        int llFid = fr.getFiducialId();
                        if(llFid == BLUE_ALLIANCE) {
                            fiducialId = llFid;
                        }

                    }

                    //   telemetry.addLine("in auto focus method-outside if-1:" + fiducialId);
                    if(fiducialId == BLUE_ALLIANCE ) {
                        //      telemetry.addLine("in auto focus method-if-2");
                        double currentAngle = getAngleFromEncoder();
                        double targetYaw = result.getTx(); // getYaw();

                        // Reads turret position
                        int turretTicks = turret.getCurrentPosition();
                        double turretAngle = degreesPerTick * turretTicks;

                        // Calculate target angle and clamp
                        double targetTurretAngle = turretAngle - targetYaw;//turretAngle + targetYaw;
                        targetTurretAngle = Math.max(minTurretDegrees, Math.min(maxTurretDegrees, targetTurretAngle));

                        // Convert degrees back to encoder ticks
                        double targetTicks = targetTurretAngle / degreesPerTick;

                        // Set the turret motor to the target (PID preferred)
                        setTurretPosition((int)targetTicks);
                    }

                }
            }

        private double getAngleFromEncoder() {
            int encoderPos = turret.getCurrentPosition();
            return encoderPos / TICKS_PER_DEGREE;
        }
        private void setTurretPosition(int targetTicks) {
            turret.setTargetPosition(targetTicks - 10);
            turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //turret.setVelocity(400);

            turret.setPower(0.5);
//            telemetry.addLine("in auto focus method");
//            telemetry.update();

            // while(turret.isBusy()) {
            // wait till turret is done turning
            //}
        }

        public Action autoResetLimelightFocus() {
            return new AutoResetLimelight();
        }

        public class SpinUpShooter implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                st.setVelocity(800);
                return false;
            }
        }

        public Action spinUpShooter() {
            return new SpinUpShooter();
        }

        public class KickerLogic implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                st.setVelocity(1550);
                sleep(500);

                kicker1.setPosition(0.32); // 0.31 kick 1
                sleep(280);

                kicker1.setPosition(0.72); // back to initial position

                // --------------------------------------------------
                // ball-2

                st.setVelocity(1550); // ball-3
                sleep(200);

                kicker2.setPosition(0.52); // 0.53 // kick 2
                sleep(280);

                kicker2.setPosition(0.13); // back to initial position

                // --------------------------------------------------
                // ball-3

                st.setVelocity(1555); //1550);
                sleep(200);

                kicker3.setPosition(0.08); // 0.07 // kick 3
                sleep(280);

                kicker3.setPosition(0.46); // back to initial position
                return false;
            }
        }

        public Action kickUpInitial(){
            return new KickerLogic();
        }
    }

    public class Intake {
        public DcMotorEx fintake;
        private double veloIntake = 1250.0;

        public Intake(HardwareMap hardwareMap) {
            fintake = hardwareMap.get(DcMotorEx.class, "fintake");
            fintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            fintake.setDirection(DcMotorEx.Direction.FORWARD); // REVERSE);
            fintake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public class StartFIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fintake.setVelocity(veloIntake);
                return false;
            }
        }

        public Action startFIntake() {
            return new StartFIntake();
        }

        public class StopFIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fintake.setVelocity(veloIntake);
                return false;
            }
        }

        public Action stopFIntkae() {
            return new StopFIntake();
        }


    }







}
