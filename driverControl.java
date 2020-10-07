package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Driver;

@TeleOp(name="driverControl", group="Linear Opmode")
//@Disabled

public class DriverControl3 extends LinearOpMode {

// declare Op mode members
        private ElapsedTime runtime = new ElapsedTime();
    //Drive train
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    //shooter things
    private DcMotor leftSpin = null;
    private DcMotor rightSpin = null;
    // shooter angle
    private DcMotor angleLift = null;
    // Ring Collector Spinners
    public CRServo adjustSpinny = null;
    private CRServo lowSpinny = null;
    private CRServo midSpinny = null;
    private CRServo highSpinny = null;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();




//init hardware variables

        // Drive Train
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftBack"); // WHeeel
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightBack"); //Another wheel?
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");  // Maybe a wheel.
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront"); //Square...

        //shooter angle adjuster
        angleLift = hardwareMap.get(DcMotor.class, "angleLift");
        //shooter things
        leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");

        //adjust Angle of spinny thing
        adjustSpinny =hardwareMap.get(CRServo.class, "adjustSpinny");

        //spinny things
        lowSpinny = hardwareMap.get(CRServo.class, "lowSpinny");
        midSpinny = hardwareMap.get(CRServo.class, "midSpinny");
        highSpinny = hardwareMap.get(CRServo.class, "highSpinny");

        // Setting the directions of motors

        //Drive train
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        //Servos
        lowSpinny.setDirection(CRServo.Direction.FORWARD);  //maybe gotta change ;~;. yeah probably ;~~;
        midSpinny.setDirection(CRServo.Direction.FORWARD);
        highSpinny.setDirection(CRServo.Direction.FORWARD);
        angleLift.setDirection(CRServo.Direction.FORWARD);






// Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


// declare speeds
        boolean slow = false;
        boolean bDown = false;

        //declare spinner mode
        boolean rightBumpDown = false;
        boolean spinnersMode = false;

        //declare shooter mode
        boolean yDown = false;
        boolean shooterMode = false;

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

// shooter mode on/off
            if (gamepad2.y == true && yDown != true) {
                yDown = true;
                shooterMode = !shooterMode;
            }
            if (gamepad2.y == false && yDown == true) {
                yDown = false;
            }
// speed checker
            if (gamepad1.b == true && bDown != true) {
                bDown = true;
                slow = !slow;
            }
            if (gamepad1.b == false && bDown == true) {
                bDown = false;
            }

            // ring collector spinners on/off
            if (gamepad2.right_bumper == true && rightBumpDown != true) {
                rightBumpDown = true;
                spinnersMode = !spinnersMode;
            }
            if (gamepad2.right_bumper == false && rightBumpDown == true) {
                rightBumpDown = false;
            }


        }

    // find numbers for turning and stuff
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            //controls angle lift
            if (gamepad2.left_bumper == true) {
            adjustTimer aT = new adjustTimer();
            aT.setDriverControl(this);
            Thread angle = new Thread(aT);
            angle.start();



            }

            if (shooterMode) {
                leftSpin.setPower(1.0);
                rightSpin.setPower(1.0);
            }
            else {
                leftSpin.setPower(0.0);
                rightSpin.setPower(0.0);
            }

            if (spinnersMode) {
                lowSpinny.setPower(1.0);
                midSpinny.setPower(1.0);
                highSpinny.setPower(1.0);
            }
            else {
                lowSpinny.setPower(0.0);
                midSpinny.setPower(0.0);
                highSpinny.setPower(0.0);
            }

    // change speed * 0.25 or not. make it drive.
            if (slow) {
                leftFrontDrive.setPower(v1 * 0.3);
                rightFrontDrive.setPower(v2 * 0.3);
                leftRearDrive.setPower(v3 * 0.3);
                rightRearDrive.setPower(v4 *0.3);

            }
            else
            {
                leftFrontDrive.setPower(v1);
                rightFrontDrive.setPower(v2);
                leftRearDrive.setPower(v3);
                rightRearDrive.setPower(v4);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
           if (slow) telemetry.addData("motor speed: ", "slow");
           else telemetry.addData("motor speed: ", "fast");
            telemetry.update();

        }
    }

    class adjustTimer implements Runnable {
    DriverControl3 dc3 = null;
    public void setDriverControl(DriverControl3 input) {
        dc3 = input;
    }
    public void run() {

//turn on servo
        if(dc3!=null) {
            dc3.adjustSpinny.setPower(0.5);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException ie) {

            }
            //turn servo off
            dc3.adjustSpinny.setPower(0.0);
        }
    }
}
