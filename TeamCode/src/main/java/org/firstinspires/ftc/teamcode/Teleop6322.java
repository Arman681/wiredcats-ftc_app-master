package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Arman on 9/13/2016.
 */

@TeleOp(name="Teleop6322", group="Opmode")

public class Teleop6322 extends OpMode {

    @Override
    public void init(){

        DcMotor FrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor FrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor BackRight = hardwareMap.dcMotor.get("br");
        DcMotor BackLeft = hardwareMap.dcMotor.get("bl");

        DcMotor[] motor = {FrontRight, FrontLeft, BackRight, BackLeft};

    }

    @Override
    public void loop(){

    }
}
