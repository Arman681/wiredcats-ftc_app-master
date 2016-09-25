package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Arman on 9/13/2016.
 */

@TeleOp(name="Teleop6322", group="Opmode")

public class Teleop6322 extends OpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //CRServo rightPusher;
    //CRServo leftPusher;

    //final DcMotor[] driveTrain = {FrontRight, FrontLeft, BackRight, BackLeft};
    @Override
    public void init() {

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop(){

        float lefty1 = -gamepad1.left_stick_y;
        float righty1 = -gamepad1.right_stick_y;

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
