package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * Created by Arman on 9/13/2016.
 */

@TeleOp(name="Teleop6322", group="Opmode")

public class Teleop6322 extends OpMode {

    ElapsedTime runtime1 = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();
    ElapsedTime runtime3 = new ElapsedTime();

    //Drive Train Motor Declarations
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //Shooting Mechanism Motor Declarations
    DcMotor right;
    DcMotor left;

    //Intake Motor Declaration
    DcMotor intake;

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    //Optical Distance Sensor Declaration
    OpticalDistanceSensor ODSleft;
    OpticalDistanceSensor ODSright;

    //Continuous Rotation Servo Declarations
    CRServo rightPusher;
    CRServo leftPusher;

    int c1 = 0;     //Left CRS Counter
    int c2 = 0;     //Right CRS Counter
    int c3 = -1;    //Shooter Counter
    int c4 = -1;    //Intake Motor Out Counter
    int c5 = -1;    //Intake Motor In Counter
    @Override
    public void init() {

        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");

        //Shooting Mechanism Motors
        right = hardwareMap.dcMotor.get("r");
        left = hardwareMap.dcMotor.get("l");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        //Optical Distance Sensors
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");

        //Continuous Rotation Servos
        rightPusher = hardwareMap.crservo.get("rp");
        leftPusher = hardwareMap.crservo.get("lp");

        //Intake Motor(s)
        intake = hardwareMap.dcMotor.get("in");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        float lefty1 = -gamepad1.left_stick_y;
        float righty1 = -gamepad1.right_stick_y;

        //Drive Train
        if (lefty1 < -.2 || lefty1 > .2) {
            FrontLeft.setPower(lefty1);
            BackLeft.setPower(lefty1);
        }
        else {
         for (int i = 1; i > .0001; i *= .1) {
             FrontLeft.setPower(lefty1*i);
             BackLeft.setPower(lefty1*i);
         }
        }
        if (righty1 < -.2 || righty1 > .2) {
            FrontRight.setPower(righty1);
            BackRight.setPower(righty1);
        }
        else {
            for (int i = 1; i > .0001; i *= .1) {
                FrontRight.setPower(righty1*i);
                BackRight.setPower(righty1*i);
            }
        }

        //Left Continuous Rotation Servo
        if (gamepad1.x && c1 == 0) {
            runtime1.reset();
            leftPusher.setPower(-1.0);
            c1 = 1;
        }
        else if (!gamepad1.x && c1 == 1)
            c1 = 2;
        else if (gamepad1.x && c1 == 2) {
            runtime1.reset();
            leftPusher.setPower(1.0);
            c1 = 3;
        }
        else if (!gamepad1.x && c1 == 3)
            c1 = 0;
        if (runtime1.time() > 2) {
            leftPusher.setPower(0);
        }

        //Right Continuous Rotation Servo
        if (gamepad1.b && c2 == 0) {
            runtime2.reset();
            rightPusher.setPower(1.0);
            c2 = 1;
        }
        else if (!gamepad1.b && c2 == 1)
            c2 = 2;
        else if (gamepad1.b && c2 == 2) {
            runtime2.reset();
            rightPusher.setPower(-1.0);
            c2 = 3;
        }
        else if (!gamepad1.b && c2 == 3)
            c2 = 0;
        if (runtime2.time() > 2) {
            rightPusher.setPower(0);
        }

        //Shooting Mechanism Motors Function
        if (gamepad1.dpad_up) {
            c3 *= -1;
            sleep(250);
        }
        if (c3 == 1) {
            right.setPower(1.0);
            left.setPower(1.0);
        }
        else if (c3 == -1) {
            int s = 1;
            while (right.getPower() > 0 && left.getPower() > 0){
                s *= 0.9;
                right.setPower(s);
            }
        }

        //Intake Motor Function Out
        if (gamepad1.dpad_right && c5 != 1) {
            c4 *= -1;
            sleep(300);
        }
        if (c4 == 1)
            intake.setPower(-1.0);
        else if (c4 == -1)
            intake.setPower(0);

        //Intake Motor Function In
        if (gamepad1.dpad_left && c4 != 1) {
            c5 *= -1;
            sleep(300);
        }
        if (c5 == 1)
            intake.setPower(1.0);
        else if (c5 == -1)
            intake.setPower(0);

        /*if (gamepad1.y)
            rightPusher.setDirection(DcMotorSimple.Direction.FORWARD);
        else if (gamepad1.b)
            rightPusher.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            rightPusher.setPower(0.0);
        if (gamepad1.x)
            leftPusher.setDirection(DcMotorSimple.Direction.FORWARD);
        else if (gamepad1.a)
            leftPusher.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            leftPusher.setPower(0.0);*/
    }
}
