package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * Created by SethHorwitz on 10/29/16.
 */

@TeleOp(name="Teleop10366", group ="Opmode")

public class Teleop10366 extends OpMode {

    ElapsedTime runtime1 = new ElapsedTime();

    //Drive Train Motor Declarations
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //Shooting Mechanism Motor Declarations
    DcMotor r;
    DcMotor l;

    //Continuous Rotation Servo + Reg. Servo Declarations
    CRServo Catapult;
    Servo Left;
    Servo Right;

    //Intake Motor Declaration
    DcMotor intake;

    //Lift Motor Declaration
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

        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        Catapult = hardwareMap.crservo.get("c");
        Left = hardwareMap.servo.get("L");
        Right = hardwareMap.servo.get("R");
        Catapult.setDirection(CRServo.Direction.REVERSE);
        Right.setDirection(Servo.Direction.REVERSE);

        //Shooting Mechanism Motors
        r = hardwareMap.dcMotor.get("r");
        l = hardwareMap.dcMotor.get("l");
        r.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake Motor
        intake = hardwareMap.dcMotor.get("in");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift Motor
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

        //Drive Train
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

        //Winch Servo Function
        if (gamepad1.x)
            Catapult.setPower(1);
        else if (gamepad1.b)
            Catapult.setPower(0);
        else
            Catapult.setPower(0.5);

        //Shooting Mechanism Motors Function
        if (gamepad2.y && c4 == 0) {
            r.setPower(-1.0);
            l.setPower(-1.0);
            c4 = 1;
        }
        else if (!gamepad2.y && c4 == 1) {
            c4 = 2;
        }
        else if (gamepad2.y && c4 == 2) {
            r.setPower(0);
            l.setPower(0);
            c4 = 3;
        }
        else if (!gamepad2.y && c4 == 3)
            c4 = 0;

        //Lift Mechanism Function
        if (gamepad2.x)
            lift.setPower(0.25);
        else if (gamepad2.b)
            lift.setPower(-1.0);
        else
        lift.setPower(0);


        telemetry.addData("Catapult Power: " + Catapult.getPower(), null);

    }
}