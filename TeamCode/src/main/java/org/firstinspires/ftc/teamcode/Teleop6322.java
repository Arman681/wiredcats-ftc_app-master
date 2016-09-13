package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Arman on 9/13/2016.
 */

@TeleOp(name="Teleop6322", group="Opmode")

public class Teleop6322 extends OpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //DcMotor[] motor = {FrontRight, FrontLeft, BackRight, BackLeft};

    float lefty1 = gamepad1.left_stick_y;
    float righty1 = gamepad1.right_stick_y;

    @Override
    public void init(){

        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");

    }

    @Override
    public void loop(){

        FrontLeft.setPower(lefty1);
        FrontRight.setPower(righty1);
        BackLeft.setPower(lefty1);
        BackRight.setPower(righty1);

    }
}
