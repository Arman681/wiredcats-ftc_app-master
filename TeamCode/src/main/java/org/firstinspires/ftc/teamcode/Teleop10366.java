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

    //Color Sensor Buttons Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    //Color Sensor White Line Declaration
    ColorSensor CSbottom;

    //Button Pusher Servo Declarations
    CRServo rightPusher;
    CRServo leftPusher;

    //Conveyor Belt Servo Declaration
    CRServo Conveyor;

    int c1 = 0;     //Left CRS Counter
    int c2 = 0;     //Right CRS Counter
    int c3 = 0;     //Shooter Counter
    int c4 = -1;    //Intake Motor Out Counter
    int c5 = -1;    //Intake Motor In Counter
    double z1 = 0.05; //Right and Left Motors deceleration Counter
    double z2 = 0.05; //Right and Left Motors acceleration Counter
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

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        //Continuous Rotation Servos
        rightPusher = hardwareMap.crservo.get("rp");
        leftPusher = hardwareMap.crservo.get("lp");

        //Intake Motor
        intake = hardwareMap.dcMotor.get("in");
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
        if (righty1 < -.2 || righty1 > .2) {
            FrontRight.setPower(righty1);
            BackRight.setPower(righty1);
        }

        //Intake
        if (gamepad1.y) {
            intake.setPower(1.0);
        }

        //Shooting Mechanism Motors Function
        if (gamepad2.y) {
            right.setPower(1.0);
            left.setPower(1.0);
        }

    }
}
