package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Arman on 9/13/2016.
 */

@TeleOp(name="Teleop6322", group="Opmode")

public class Teleop6322 extends OpMode {

    //Drive Train Motor Declarations
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    //Optical Distance Sensor Declaration
    OpticalDistanceSensor ODSleft;
    OpticalDistanceSensor ODSright;

    //Continuous Rotation Servo Declarations
    CRServo rightPusher;
    CRServo leftPusher;

    int c1 = 0;
    int c2 = 0;
    @Override
    public void init() {

        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csleft");
        CSright = hardwareMap.colorSensor.get("csright");

        //Optical Distance Sensors
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");

        //Continuous Rotation Servos
        rightPusher = hardwareMap.crservo.get("rightPusher");
        leftPusher = hardwareMap.crservo.get("leftPusher");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop(){

        float lefty1 = -gamepad1.left_stick_y;
        float righty1 = -gamepad1.right_stick_y;

        CSright.enableLed(true);
        CSleft.enableLed(true);

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
        if (righty1 < -.2 || righty1 > .2){
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
            leftPusher.setPower(-1.0);
            c1++;
        }
        else if (!gamepad1.x && c1 == 1) {
            c1++;
            leftPusher.setPower(0);
        }
        else if (gamepad1.x && c1 ==2) {
            leftPusher.setPower(1.0);
            c1++;
        }
        else if (!gamepad1.x && c1 == 3) {
            c1 = 0;
            leftPusher.setPower(0);
        }

        //Right Continuous Rotation Servo
        if (gamepad1.b && c2 == 0) {
            rightPusher.setPower(1.0);
            c2++;
        }
        else if (!gamepad1.b && c2 == 1) {
            c2++;
            rightPusher.setPower(0);
        }
        else if (gamepad1.b && c2 ==2) {
            rightPusher.setPower(-1.0);
            c2++;
        }
        else if (!gamepad1.b && c2 == 3) {
            c2 = 0;
            rightPusher.setPower(0);
        }


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
