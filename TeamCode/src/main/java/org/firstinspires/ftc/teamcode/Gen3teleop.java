package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Arman on 10/29/2016.
 */

public class Gen3teleop extends OpMode {

    DcMotor frontleft;
    DcMotor backleft;
    DcMotor frontright;
    DcMotor backright;

    @Override
    public void init() {

        frontleft = hardwareMap.dcMotor.get("fl");
        backleft = hardwareMap.dcMotor.get("bl");
        frontright = hardwareMap.dcMotor.get("fr");
        backright = hardwareMap.dcMotor.get("br");

        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;

        frontleft.setPower(leftY);
        backleft.setPower(leftY);
        backright.setPower(rightY);
        frontright.setPower(rightY);

    }
}
