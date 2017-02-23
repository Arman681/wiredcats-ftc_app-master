package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by SethHorwitz on 11/18/16.
 */
//@Autonomous(name="RedAuto10366", group="Autonomous")

public class RedAuto10366 extends LinearOpMode {

    ElapsedTime runtime1 = new ElapsedTime(); //Counter for shoot();
    ElapsedTime runtime2 = new ElapsedTime(); //Counter for determineRedSide();

    //Drive Train Motor Declarations
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    final DcMotor[] driveTrain = new DcMotor[4];

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

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        CSright.enableLed(true);
        CSleft.enableLed(true);
        CSleft.enableLed(false);
        CSright.enableLed(false);


        waitForStart(); //Autonomous begins when play button is pressed on the Driver Station Phone

        //Shooting 2 Balls
        Catapult.setPower(1); //Sets catapult winch servo backwards to recharge forward rotation
        moveByTime(-0.5, 3000); //Move backwards at full speed for 3 seconds
        shoot(1.0, 3000, 1000); //Shoots particles at full power for 3 seconds and starts catapult after 1 second

        //Claim Red Beacon 1
        moveByTime(0.5, 1000); //Move forwards at full speed for 1 second
        turnByTime(-0.5, 750); //Turns counter-clock-wise at half speed for three-quarters of a second to make 45-degree turn
        moveByTime(-0.5, 2000); //Move backwards at full speed for 2 seconds
        turnByTime(0.5, 2250); //Turns clock-wise at half speed for one-and-a-half seconds to make 135-degree turn
        moveByTime(0.25, 500); //Move forward at one-quarter speed for half a second to get closer to beacon
        goForButton(); //Determines red side of beacon and hits button on that side

        //Claim Red Beacon 2
        moveByTime(-0.5, 1500); //Move backwards at half speed for one-and-a-half seconds
        turnByTime(-0.5, 1500); //Turns counter-clock-wise at half speed for one-and-a-half seconds to make 90-degree turn
        moveByTime(-0.5, 2500); //Move backwars at full speed for two-and-a-half seconds
        turnByTime(0.5, 1500); //Turns clock-wise at half speed for one-and-a-half seconds to make 90-degree turn
        moveByTime(0.25, 500); //Move forward at one-quarter speed for half a second to get closer to beacon
        goForButton(); //Determines red side of beacon and hits button on that side

        //Claim Red Cap Ball
        moveByTime(-0.5, 1500); //Move backwards at half speed for one-and-a-half seconds
        turnByTime(-0.5, 2250); //Turns counter-clock-wise at half speed for one-and-a-half seconds to make 135-degree turn
        moveByTime(0.5, 3000); //Move forwards at full speed for three seconds
        turnByTime(0.5, 750); //Turns clock-wise at half speed for three-quarters of a second to make 45-degree turn
        moveByTime(0.75, 1500); //Move forwards at full speed for 1 second

        //Park at Red Corner Vortex
        turnByTime(0.5, 750); //Turns clock-wise at half speed for three-quarters of a second to make 45-degree turn
        moveByTime(0.5, 1000); //Move forwards at full speed for 1 second
        moveByTime(0.25, 2000); //Move forwards at one-quarter speed for 2 seconds
        stopAllMotors();
    }

    public void goForButton() throws InterruptedException {

        boolean dec = false;
        String determinedSide;
        determinedSide = determineRedSide();

        while(!dec) {

            if (determinedSide == "left") {

                turnByTime(-0.5, 750); //Turns counter-clock-wise at half speed for three-quarters of a second to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for half a second
                turnByTime(0.5, 750); //Turns clock-wise at half speed for three-quarters of a second to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for half a second
                dec = true;
            }
            else if (determinedSide == "right") {

                turnByTime(0.5, 750); //Turns clock-wise at half speed for three-quarters of a second to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for half a second
                turnByTime(-0.5, 750); //Turns counter-clock-wise at half speed for three-quarters of a second to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for half a second
                dec = true;
            }
            else if (determinedSide == "null") {
                moveByTime(0.25, 500);
                determinedSide = determineRedSide();
            }
        }
    }

    public String determineRedSide() throws InterruptedException {

        boolean dec = false;
        int c1 = 0; //Iterations counter
        String c = "";
        runtime2.reset();

        while(!dec) {
            if ((CSleft.red() * 8 > CSleft.blue() * 8) && (CSright.blue() * 8 > CSright.red() * 8)) {
                c = "left";
                dec = true;
            }
            else if ((CSleft.blue() * 8 > CSleft.red() * 8) && (CSright.red() * 8 > CSright.blue() * 8)) {
                c = "right";
                dec = true;
            }
            else if (runtime2.time() > 4000) {
                c = "null";
                dec = true;
            }
            c1++;
            telemetry.addData("LED", true ? "On" : "Off");
            telemetry.addData("Red  ", CSleft.red()*8);
            telemetry.addData("Blue ", CSleft.blue()*8);
            telemetry.addData("Iterations: " + c1, null);
            telemetry.update();

        }

        return c;
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

    public void shoot(double power, int targetTime, int catapultDelay) throws InterruptedException {

        runtime1.reset();
        if (runtime1.time() < targetTime) {
            r.setPower(power);
            l.setPower(power);
        }
        else if (runtime1.time() > targetTime) {
            r.setPower(0);
            l.setPower(0);
        }
        if (runtime1.time() > catapultDelay)
            Catapult.setPower(0);
    }

    public void stopAllMotors() throws InterruptedException {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        r.setPower(0);
        l.setPower(0);
    }

}
