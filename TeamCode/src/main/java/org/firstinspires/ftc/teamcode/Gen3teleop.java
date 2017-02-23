package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * Created by Arman on 10/29/2016.
 */

//@TeleOp(name="Gen3Teleop", group="Opmode")
public class Gen3teleop extends OpMode {

    ElapsedTime runtime1 = new ElapsedTime(); //Left Button Pusher Timer
    ElapsedTime runtime2 = new ElapsedTime(); //Right Button Pusher Timer
    ElapsedTime runtime3 = new ElapsedTime(); //Fork Lock Timer 2.a
    ElapsedTime runtime4 = new ElapsedTime(); //Fork Lock Timer 2.y

    int c1 = 0;     //Left Button Pusher Counter
    int c2 = 0;     //Right Button Pusher Counter
    int c3 = 0;     //Shooter Counter
    int c4 = 0;     //Intake Motor In Counter
    int c5 = 0;     //Intake Motor Out Counter
    int c6 = 0;     //Fork Lock 2.a
    int c7 = 0;     //Fork Lock 2.y
    double z1 = 0.05; //Right and Left Motors deceleration Counter
    double z2 = 0.05; //Right and Left Motors acceleration Counter

    //Drive Train Motor Declarations
    DcMotor dtleft, dtright;

    //Shooting Mechanism Motor Declarations
    DcMotor right, left;

    //Particle System Motor Declarations
    DcMotor intake;

    //Lift System Motor Declarations
    DcMotor winch, conveyor;

    //Fork Lock Servo Declaration
    CRServo lock;

    //Servo Button Pusher Declaration
    CRServo rightPusher, leftPusher;

    @Override
    public void init() {

        //Drive Train Motors
        dtright = hardwareMap.dcMotor.get("dtr");
        dtleft = hardwareMap.dcMotor.get("dtl");
        dtright.setDirection(DcMotor.Direction.REVERSE);

        //Shooting Mechanism Motors
        right = hardwareMap.dcMotor.get("r");
        left = hardwareMap.dcMotor.get("l");

        //Intake and Conveyor Motors
        intake = hardwareMap.dcMotor.get("i");
        conveyor = hardwareMap.dcMotor.get("c");
        intake.setDirection(DcMotor.Direction.REVERSE);

        conveyor.setPower(0);

        //Lift System Motors
        winch = hardwareMap.dcMotor.get("w");

        //Button Pusher Servos
        rightPusher = hardwareMap.crservo.get("rp");
        leftPusher = hardwareMap.crservo.get("lp");

        //Fork Lock Servo
        lock = hardwareMap.crservo.get("k");
    }

    @Override
    public void loop() {

        //Drive Train Functionality
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        dtleft.setPower(leftY);
        dtright.setPower(rightY);

        //Left Button Pusher Servo
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

        //Right Button Pusher Servo
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
        if (runtime2.time() > 2)
            rightPusher.setPower(0);

        //Shooting Mechanism Motors Function
        if (gamepad2.dpad_up && c3 == 0) {
            right.setPower(1.0);
            left.setPower(1.0);
            c3 = 1;
        }
        else if (!gamepad2.dpad_up && c3 == 1)
            c3 = 2;
        else if (gamepad2.dpad_up && c3 == 2) {
            right.setPower(0);
            left.setPower(0);
            c3 = 3;
        }
        else if (!gamepad2.dpad_up && c3 == 3)
            c3 = 0;

        //Winch Function
        if (gamepad2.b)
            winch.setPower(1.0);
        else if (gamepad2.x)
            winch.setPower(-1.0);
        else
            winch.setPower(0);

        //Intake Motor Function In
        if (gamepad2.dpad_left && c4 == 0) {
            intake.setPower(1.0);
            c4 = 1;
        }
        else if (!gamepad2.dpad_left && c4 == 1)
            c4 = 2;
        else if (gamepad2.dpad_left && c4 == 2) {
            intake.setPower(0);
            c4 = 3;
        }
        else if (!gamepad2.dpad_left && c4 == 3)
            c4 = 0;

        //Intake Motor Function Out
        if (gamepad2.dpad_right && c5 == 0) {
            intake.setPower(-1.0);
            c5 = 1;
        }
        else if (!gamepad2.dpad_right && c5 == 1)
            c5 = 2;
        else if (gamepad2.dpad_right && c5 == 2) {
            intake.setPower(0);
            c5 = 3;
        }
        else if (!gamepad2.dpad_right && c5 == 3)
            c5 = 0;

        //Conveyor Belt Function
        if (gamepad2.right_trigger == 1)
            conveyor.setPower(-1.0);
        else if (gamepad2.left_trigger == 1)
            conveyor.setPower(1.0);
        else
            conveyor.setPower(0);

        //Fork Lock Function 2.a
        if (gamepad2.a && c6 == 0) {
            runtime3.reset();
            lock.setPower(-1.0);
            c6 = 1;
        }
        else if (!gamepad2.a && c6 == 1) {
            c6 = 2;
        }
        else if (!gamepad2.a && c6 == 2) {
            if (runtime3.time() > 2)
                lock.setPower(0);
            c6 = 3;
        }
        else if (gamepad2.a && c6 == 3) {
            runtime3.reset();
            lock.setPower(1.0);
            c6 = 4;
        }
        else if (!gamepad2.a && c6 == 4) {
            c6 = 5;
        }
        else if (!gamepad2.a && c6 == 5) {
            if (runtime3.time() > 2)
                lock.setPower(0);
            c6 = 0;
        }

        telemetry.addData("Intake Power: " + intake.getPower(), null);
        telemetry.addData("Intake Counter: " + c4, null);
    }
}
