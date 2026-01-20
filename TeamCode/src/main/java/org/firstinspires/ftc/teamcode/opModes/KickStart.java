/*
package org.firstinspires.ftc.teamcode.opModes;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "KickTester", group="Decode")
public class KickStart extends LinearOpMode {
    
    private DcMotorEx st;
    private Servo kicker1, kicker2, kicker3;
    
    @Override
    public void runOpMode() {
        initializeShooterMotors();
        initializeKickers();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        st.setVelocity(1460);
        telemetry.addData("shooter-1", st.getVelocity());
        telemetry.update();
        
        
        while (opModeIsActive()) {
            telemetry.addData("shooter-2", st.getVelocity());
            telemetry.update();
            if (gamepad1.y) {
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // distance: 45 inches
                // --------------------------------------------------
                // ball-1
                
                st.setVelocity(1460);
                
                sleep(400);
                
                kicker1.setPosition(0.31); //0.6); // KICKING_POSITION);
                sleep(250);
                
                kicker1.setPosition(0.72);
                
                // --------------------------------------------------
                
                // --------------------------------------------------
                // ball-2
                
                sleep(175);
                st.setVelocity(1490);
                sleep(175);
                
                kicker2.setPosition(0.53);//0.55 // 0.7
                sleep(200);
                
                kicker2.setPosition(0.13);//0.3);
                
                
                // --------------------------------------------------
                
                // --------------------------------------------------
                // ball-3
                
                st.setVelocity(1500);
                sleep(400);
                
                kicker3.setPosition(0.07);
                sleep(250);
                
                kicker3.setPosition(0.46);
                
                // --------------------------------------------------
                
                
            } else if (gamepad1.a) {
                // working code
                st.setVelocity(800);
                kicker1.setPosition(0.71); //0.13); //RESTING_POSITION);
                kicker2.setPosition(0.15);//0.3);
                kicker3.setPosition(0.45);
            } 
            
            //sleep(10); // Small delay for smooth operation
        }
    }
    
      private double angleToPos(double angle) {
        angle = Math.max(20.0, Math.min(260.0, angle));
        
        return angle / 300.0;
    }
    

     private void initializeShooterMotors() {
        st = hardwareMap.get(DcMotorEx.class, "shooter"); // shooter top motor
        st.setDirection(DcMotorEx.Direction.FORWARD);
        
        // Set motor to RUN_USING_ENCODER mode for velocity control
        st.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        
        // Set zero power behavior
        st.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
    }
    
    private void initializeKickers() {
            */
/*
    private static final double RESTING_POSITION2 = 0.001;
    private static final double MAX_POSITION2 = 1.0;
    private static final double KICKING_POSITION2 = 0.5; // Center/135°
    *//*

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
    
    
}
*/
