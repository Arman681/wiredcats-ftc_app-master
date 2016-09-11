package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jb on 9/9/16.
 */

@TeleOp(name="bae", group="Opmode")

public class bae extends OpMode{

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor lift;

    @Override
    public void init() {

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        lift = hardwareMap.dcMotor.get("lift");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

    	BL.setPower(-gamepad1.left_stick_y);
    	BR.setPower(-gamepad1.right_stick_y);
    	FL.setPower(-gamepad1.left_stick_y);
    	FR.setPower(-gamepad1.right_stick_y);

    	lift.setPower(-gamepad2.right_stick_y);
    }

}
