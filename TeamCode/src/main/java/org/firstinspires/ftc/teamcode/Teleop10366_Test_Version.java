package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Carol on 1/17/2017.
 */


//@TeleOp(name="Teleop10366_Test_Version", group ="Opmode")

public class Teleop10366_Test_Version extends OpMode {

 //comment to push file *

    ElapsedTime runtime1 = new ElapsedTime();



        //  Drive Train Motor Declarations

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;


            // Shooting Mechanism Motor Declarations

    DcMotor r;
    DcMotor l;


            // Continuous Rotation Servo + Reg. Servo Declarations

    CRServo Catapult;
    Servo Left;
    Servo Right;

    double catapultClockwise = 1;
    double catapultCClockwise = -1;
    double catapultStop = 0;



    // Intake Motor Declaration

    DcMotor intake;


            // Lift Motor Declaration

    DcMotor lift;

    ColorSensor csl;
    ColorSensor csr;
    GyroSensor gyro;

    int c2 = 0; //CRS Counter
    int c3 = 0; //Intake Motor Counter In
    int c4 = 0; //Shooter Motors Counter
    int c5 = 0; //Intake Motor Counter Out


    @Override
    public void init() {


                // Drive Train Motors

        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);


                // Servos

        Catapult = hardwareMap.crservo.get("c");
        Left = hardwareMap.servo.get("L");
        Right = hardwareMap.servo.get("R");
        Right.setDirection(Servo.Direction.REVERSE); //not sure why this is here


                // Shooting Mechanism Motors

        r = hardwareMap.dcMotor.get("r");
        l = hardwareMap.dcMotor.get("l");
        r.setDirection(DcMotorSimple.Direction.REVERSE);


                // Intake Motor

        intake = hardwareMap.dcMotor.get("in");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


                // Lift Motor

        lift = hardwareMap.dcMotor.get("lift");

        csl = hardwareMap.colorSensor.get("csl");
        csr = hardwareMap.colorSensor.get("csr");
        gyro = hardwareMap.gyroSensor.get("g");
    }

    @Override
    public void loop() {

        Right.setPosition(-1.0);
        Left.setPosition(-1.0);

        float lefty1 = -gamepad1.left_stick_y;
        float righty1 = -gamepad1.right_stick_y;

                // Drive Train  *** Left  WheelsGamepad 1 Left Stick
                //              *** Right WheelsGamepad 1 Right Stick

        if (lefty1 < -.2 || lefty1 > .2) {
            FrontLeft.setPower(lefty1);
            BackLeft.setPower(lefty1);
        }
        else {
            for (int i = 1; i > .0001; i *= .1) {
                FrontLeft.setPower(lefty1 * i);
                BackLeft.setPower(lefty1 * i);
            }
        }
        if (righty1 < -.2 || righty1 > .2) {
            FrontRight.setPower(righty1);
            BackRight.setPower(righty1);
        }
        else {
            for (int i = 1; i > .0001; i *= .1) {
                FrontRight.setPower(righty1 * i);
                BackRight.setPower(righty1 * i);
            }
        }

                // Intake In        *** Gamepad 1 Button A ***

        if (gamepad1.a && c3 == 0) {
            intake.setPower(1.0);
            c3 = 1;
        }
        else if (!gamepad1.a && c3 == 1)
            c3 = 2;
        else if (gamepad1.a && c3 == 2) {
            intake.setPower(0);
            c3 = 3;
        }
        else if (!gamepad1.a && c3 == 3)
            c3 = 0;

                // Intake Out       *** Gamepad 1 Button Y ***

        if (gamepad1.y && c5 == 0) {
            intake.setPower(-1.0);
            c5 = 1;
        }
        else if (!gamepad1.y && c5 == 1)
            c5 = 2;
        else if (gamepad1.y && c5 == 2) {
            intake.setPower(0);
            c5 = 3;
        }
        else if (!gamepad1.y && c5 == 3)
            c5 = 0;

                // SPS Servo Function   *** Gamepad 2 Button X  for clockwise direction (>.5) ***
                //                      *** Gamepad 2 Button B  for counter-clockwise direction (<.5)

        /*if (gamepad2.x)  // X gamepade = clockwise = shoot
            Catapult.setPower(1);  //turn clockwise
          //else if (gamepad2.b) // B gamepad  = counter-clockwise probably ****  DO NOT USE ****
          // Catapult.setPower(-1); // Turn counter-clock-wise

        else
            Catapult.setPower(0.5); // stop servo
*/
                // Shooting Mechanism Motors *** Gamepad 2 Button Y ***
                // Function  uses toggel system

        if (gamepad2.y && c4 == 0) {  // when "Y" pressed starts motors
            c4 = 1;
        }
        else if (!gamepad2.y && c4 == 1) {  // when "Y let go keeps it where it was
            runtime1.reset();
            r.setPower(-1.0);
            l.setPower(-1.0);c4 = 2;
        }
        else if (!gamepad2.y && c4 == 2 && runtime1.time() > 1.75)
            Catapult.setPower(catapultClockwise);
        else if (gamepad2.y && c4 == 2&& runtime1.time() > 5.5) {  // when "Y pressed againt will stop motors
            r.setPower(0);
            l.setPower(0);
            Catapult.setPower(catapultStop);
            c4 = 3;
        }

        else if (!gamepad2.y && c4 == 3)  // keeps motors where the were stopped waiting for next press
            c4 = 0;



                // Cap Ball Lift Mechanism Function
                    // *** CHANGE TO USE UP/DOWN/LEFT RIGHT BUTTON ON LEFT SIDE OF JOY STICK) ***
                    //  *** CHANGE TO UP TO GO CLOCKWISE AND DOWN TO GOT COUNTER CLOCKWISE  ***

                    // ***OLD CODE***     *** Gamepad 2 Button X  clock-wise (positive Value)***
                    // ***OLD CODE***     *** Gamepad 2 Button B  Counter- clock-wise (negative Value) ***

        if (gamepad2.dpad_up)
            lift.setPower(0.25);
        else if (gamepad2.dpad_down)
            lift.setPower(-1.0);
        else
            lift.setPower(0);

        telemetry.addData("Catapult Power: " + Catapult.getPower(), null);

    }
}
