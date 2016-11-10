package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

/**
 * Created by Arman on 10/29/2016.
 */

@TeleOp(name="Gen3Teleop", group="Opmode")
public class Gen3teleop extends OpMode {

    int c3 = -1;     //Shooter Counter
    int c4 = -1;    //Intake Motor Out Counter
    int c5 = -1;    //Intake Motor In Counter
    double z1 = 0.05; //Right and Left Motors deceleration Counter
    double z2 = 0.05; //Right and Left Motors acceleration Counter

    //Drive Train Motor Declarations
    DcMotor frontleft, backleft;
    DcMotor frontright, backright;

    //Shooting Mechanism Motor Declarations
    DcMotor right, left;

    //Particle System Motor Declarations
    DcMotor intake, conveyor;

    //Servo Declaration
    Servo servo;

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
        left.setDirection(DcMotor.Direction.REVERSE);

        //Intake and Conveyor Motors
        intake = hardwareMap.dcMotor.get("i");
        conveyor = hardwareMap.dcMotor.get("c");
        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);

        //Servo
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {

        //Drive Train Functionality
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        frontleft.setPower(leftY);
        backleft.setPower(leftY);
        backright.setPower(rightY);
        frontright.setPower(rightY);

        /*if (gamepad2.dpad_up) {
            right.setPower(1.0);
            left.setPower(1.0);
        }
        else if (!gamepad2.dpad_up) {
            right.setPower(0);
            left.setPower(0);
        }*/

        //Shooting Mechanism Motors Function
        if (gamepad2.dpad_up) {
            c3 *= -1;
            sleep(300);
        }
        if (c3 == -1) {
            right.setPower(0);
            left.setPower(0);
        }
        else if (c3 == 1) {
            right.setPower(1.0);
            left.setPower(1.0);
        }

        //Intake Motor Function In
        //Counters c4 and c5 both initialized as -1
        if (gamepad2.dpad_left) {
            c4 *= -1;
            sleep(300);
        }
        if (c4 == 1)
            intake.setPower(1.0);
        else if (c4 == -1)
            intake.setPower(0);

        //Conveyor Belt Function
        if (gamepad2.dpad_right)
            conveyor.setPower(-1.0);
        else
            conveyor.setPower(0.0);

        /*Intake Motor Function Out
        if (gamepad2.dpad_right) {
            c5 *= -1;
            sleep(300);
        }
        if (c5 == 1)
            intake.setPower(1.0);
        else if (c5 == -1)
            intake.setPower(0);
        */
        telemetry.addData("c3 count: " + c3, null);
        telemetry.addData("c4 count: " + c4, null);
    }
}
