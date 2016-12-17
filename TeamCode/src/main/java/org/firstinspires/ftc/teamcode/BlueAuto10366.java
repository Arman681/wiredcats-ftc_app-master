package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by SethHorwitz on 12/17/16.
 */
@Autonomous(name="BlueAuto10366", group="Autonomous")

public class BlueAuto10366 extends LinearOpMode{

    ElapsedTime runtime1 = new ElapsedTime();

    //Drive Train Motor Declarations
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    final DcMotor[] driveTrain = new DcMotor[4];

    //Shooting Mechanism Motor Declarations
    DcMotor right;
    DcMotor left;

    //Intake Motor Declaration
    DcMotor intake;

    //Lift Motor Declaration
    DcMotor lift;

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    int bnum = 0;
    int ds2 = 2;  // additional downsampling of the image

    float hsvValues[] = {0F,0F,0F};

    //encoder constants
    static final double TAU                  = 6.283185;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES  = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);
    static final double DEGREES_TO_ENCODER_INCHES = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");

        driveTrain[0] = FrontRight;
        driveTrain[1] = FrontLeft;
        driveTrain[2] = BackRight;
        driveTrain[3] = BackLeft;

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        CSright.enableLed(true);
        CSleft.enableLed(true);
        CSleft.enableLed(false);
        CSright.enableLed(false);



        waitForStart();
        moveByTime(0.5, 3000);
        turnByTime(0.5, 1000);
        moveByTime(0, 500);
        moveByTime(0.5, 1000);


    }

    public void moveByTime(double power, int time) throws InterruptedException {

        for(DcMotor motor : driveTrain)
            motor.setPower(power);

        sleep(time);

        for(DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    public void turnByTime(double power, int time) throws InterruptedException {
        //Positive power makes robot turn right
        //Negative power makes robot turn left

        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
        sleep(time);
    }


}
