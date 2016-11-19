package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * Created by SethHorwitz on 10/29/16.
 */

@TeleOp(name="Teleop10366", group ="Opmode")

public class Teleop10366 extends OpMode {

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

    //Lift Motor Declaration
    DcMotor lift;

    int c3 = 0; //Intake Motor Counter In
    int c4 = 0; //Shooter Motors Counter
    int c5 = 0; //Intake Motor Counter Out

    @Override
    public void init() {

        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //Shooting Mechanism Motors
        right = hardwareMap.dcMotor.get("r");
        left = hardwareMap.dcMotor.get("l");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake Motor
        intake = hardwareMap.dcMotor.get("in");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift Motor
        lift = hardwareMap.dcMotor.get("lift");
    }

    @Override
    public void loop() {
        float lefty1 = -gamepad1.left_stick_y;
        float righty1 = -gamepad1.right_stick_y;

        //Drive Train
        if (gamepad1.left_stick_y == 1) {
            FrontLeft.setPower(lefty1);
            BackLeft.setPower(lefty1);
        }
        else {
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
        }
        if (gamepad1.right_stick_y == 1) {
            FrontRight.setPower(righty1);
            BackRight.setPower(righty1);
        }
        else {
            FrontRight.setPower(0);
            BackRight.setPower(0);
        }

        //Intake In
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

        //Intake Out
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

        //Shooting Mechanism Motors Function
        if (gamepad2.y && c4 == 0) {
            right.setPower(-1.0);
            left.setPower(-1.0);
            c4 = 1;
        }
        else if (!gamepad2.y && c4 == 1)
            c4 = 2;
        else if (gamepad2.y && c4 == 2) {
            right.setPower(0);
            left.setPower(0);
            c4 = 3;
        }
        else if (!gamepad2.y && c4 == 3)
            c4 = 0;

        //Add this comment right here

    }
}