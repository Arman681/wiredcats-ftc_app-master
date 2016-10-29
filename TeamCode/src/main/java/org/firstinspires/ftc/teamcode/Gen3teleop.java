package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static android.os.SystemClock.sleep;

/**
 * Created by Arman on 10/29/2016.
 */

@TeleOp(name="Gen3Teleop", group="Opmode")
public class Gen3teleop extends OpMode {

    int c3 = 0;     //Shooter Counter
    int c4 = -1;    //Intake Motor Out Counter
    int c5 = -1;    //Intake Motor In Counter
    double z1 = 0.05; //Right and Left Motors deceleration Counter
    double z2 = 0.05; //Right and Left Motors acceleration Counter

    DcMotor frontleft, backleft;
    DcMotor frontright, backright;

    DcMotor right, left;
    DcMotor intake, conveyor;

    @Override
    public void init() {

        //Drive Train Motors
        frontleft = hardwareMap.dcMotor.get("fl");
        backleft = hardwareMap.dcMotor.get("bl");
        frontright = hardwareMap.dcMotor.get("fr");
        backright = hardwareMap.dcMotor.get("br");
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);

        //Shooting Mechanism Motors
        right = hardwareMap.dcMotor.get("r");
        left = hardwareMap.dcMotor.get("l");
        right.setDirection(DcMotor.Direction.REVERSE);

        //Intake and Conveyor Motors
        intake = hardwareMap.dcMotor.get("i");
        conveyor = hardwareMap.dcMotor.get("c");
        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //Drive Train Functionality
        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;

        frontleft.setPower(leftY);
        backleft.setPower(leftY);
        backright.setPower(rightY);
        frontright.setPower(rightY);

        //Shooting Mechanism Motors Function
        if (gamepad2.dpad_up && c3 == 0) {
            c3 = 1;
        }
        else if (!gamepad2.dpad_up && c3 == 1) {
            z2 *= 0.05;
            if (z2 > 0) {
                right.setPower(z2);
                left.setPower(z2);
                sleep(500);
            }
            else {
                right.setPower(1.0);
                left.setPower(1.0);
                z2 = 0.05;
            }
            if (right.getPower() < 1 && left.getPower() < 1)
                c3 = 1;
            else
                c3 = 2;
        }
        else if (gamepad2.dpad_up && c3 == 2) {
            c3 = 3;
        }
        else if (!gamepad2.dpad_up && c3 == 3) {
            z1 *= 1.2;
            if ((1 - z1) > 0) {
                right.setPower(1 - z1);
                left.setPower(1 - z1);
            }
            else {
                right.setPower(0);
                left.setPower(0);
                z1 = 0.05;
            }
            if (right.getPower() > 0 && left.getPower() > 0)
                c3 = 3;
            else
                c3 = 0;
            sleep(500);
        }

        //Intake Motor Function In
        //Counters c4 and c5 both initialized as -1f
        if (gamepad2.dpad_left) {
            c5 *= -1;
            sleep(300);
        }
        if (c5 == 1)
            intake.setPower(1.0);
        else if (c5 == -1)
            intake.setPower(0);

        //Conveyor Motor Function In
        if (gamepad2.dpad_right) {
            c4 *= -1;
            sleep(300);
        }
        if (c4 == 1)
            intake.setPower(1.0);
        else if (c4 == -1)
            intake.setPower(0);

    }
}
