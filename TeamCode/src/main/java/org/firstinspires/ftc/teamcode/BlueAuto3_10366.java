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

import java.security.PublicKey;

/**
 * Created by Carol on 1/24/2017.
 */


@Autonomous(name="BlueAuto3_10366", group="Autonomous")

public class BlueAuto3_10366 extends LinearOpMode {  // LinearOpMode is for autonomous
   // public class BlueAuto10366_Test_Version extends LinearOpMode {

    String determinedSide;

    ElapsedTime runtime1 = new ElapsedTime(); //Counter for shoot();
    ElapsedTime runtime2 = new ElapsedTime(); //Counter for determineRedSide();

    //Drive Train Motor Declarations
    DcMotor FrontRight;  //DcMotor is a class  Frontright is an object
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    final DcMotor[] driveTrain = new DcMotor[4];  // array decloration - [4] indicates 4  allocations (motors)


    //Shooting Mechanism Motor Declarations

    DcMotor r;
    DcMotor l;


    //Continuous Rotation Servo + Lift Reg. Angular Servo Declarations

    CRServo Catapult;   //Continuous Rotation Servo
    Servo Left;         // Left lift fork locking servo
    Servo Right;        // Right lift fork locking sevro

    double catapultClockwise = 1;
    double catapultCClockwise = -1;
    double catapultStop = 0;


    //Intake Motor Declaration

    DcMotor intake;


    //Lift Motor Declaration

    DcMotor lift;


    //Color Sensor Declarations

    ColorSensor CSleft;
    ColorSensor CSright;

    int fourtyfivedegrees = 410;
    int ninetydegrees = (fourtyfivedegrees*2 + 10);
    int onethirtyfivedegree = (fourtyfivedegrees*3 + 20);

    //Encoder Constants

    //Moror Values from Andymark for NeverRest 60

    static final double TAU = 6.283185;  // = 2*pi
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES = 2.0;      // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);
    static final double DEGREES_TO_ENCODER_INCHES = 0;


    //Motor Values from Andymark for NeverRest 20

    static final double TAU_20 = 6.283185;
    static final double COUNTS_PER_MOTOR_REV_20 = 1120;     // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION_20 = 1.0;      // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES_20 = 2.0;       // For figuring circumference
    static final double COUNTS_PER_INCH_20 = (COUNTS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION_20) / (WHEEL_RADIUS_INCHES_20 * TAU_20);
    static final double DEGREES_TO_ENCODER_INCHES_20 = 0;

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
        //Catapult.setDirection(CRServo.Direction.REVERSE); // would be required if servo was mounted backwards
        Right.setDirection(Servo.Direction.REVERSE); // sets right lift fork servo to go opposite of left


        //Shooting Mechanism Motors

        r = hardwareMap.dcMotor.get("r");
        l = hardwareMap.dcMotor.get("l");
        l.setDirection(DcMotorSimple.Direction.REVERSE);


        //Intake Motor

        intake = hardwareMap.dcMotor.get("in");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        //Lift Motor

        lift = hardwareMap.dcMotor.get("lift");


        //Color Sensors

        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        CSright.enableLed(true);  // need to set on (true) and off(false) to get sensors to function correctly
        CSleft.enableLed(true);
        CSleft.enableLed(false);
        CSright.enableLed(false);

        waitForStart(); //Autonomous begins when play button is pressed on the Driver Station Phone


        //Correct Initial Counter Clock-Wise Turn

        moveByTime(-0.25, 10);  //Move Backward  at one-quarter speed for  .010 seconds  ***code to correct initial counter-clock-wise turn
        turnByTime(-0.25, 50); // Move Forward  at half speed for  .050seconds counter clock-wise ***code to correct initial clock-wise turn


        // Move into Shooting Position and Shoot 2 Particals

        moveByTime(-0.25, 1000); //move Backward at one-quarter speed for 1.250 seconds*** changed to 1000***

        // Shoot 2 Particles

        shoot(1.0, 5.5, 1.75); //Shoots particles at full power for  change  changed to 4.75 / 1.5

        Catapult.setPower(.5); //Sets catapult servo to stop


        //Claim Blue Beacon 1  ***changed quarter turn from .405 to .410

        moveByTime(0.25, 850); //Move Forward at one quarter speed for .600 changed to ***.750 to 850 after ball was inflated***
        turnByTime(0.25, fourtyfivedegrees); //Turn Clock-wise at one-quarter speed for .410 seconds(.425 seconds - .15 Seconds) to offset (initialization) to make 45-degree turn
        moveByTime(-0.25, 2200); //Move Backward at one-quarter speed for 2.200 seconds ***Battery Full charge  14.44 - 14.00** changed to 20000 from 1750
        turnByTime(-0.25, onethirtyfivedegree); //Turns Counter-clock-wise at one-quarter speed for 1.230 seconds to make 135-DEGREE (3 X 45-DEGREE) turn
        //moveByTime(-0.25, 50); // Move Backward  at one-quarter speed for .050 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward   at one-quarter speed for .050 seconds (***used to correct motor direction to go straight***)
        moveByTime(0.25, 500); // Move Forward at one-quarter speed for .650 seconds to establish initial Beacon Startion position
        moveByTime(0.25, 250); // Move Forward at one-quarter speed for .350 seconds to get closer to beacon
        goForButton(); //Determines blue side of beacon and hits button on that side


        //Claim Blue Beacon 2

        moveByTime(-0.25, 1000); //Move Backwards at half speed for 1 seconds
        turnByTime(-0.25, ninetydegrees + fourtyfivedegrees/2); //Turns Counter-clock-wise at one-quarter speed for .8 seconds to make 90-DEGREE (2 X 45-DEGREE) turn
        //turnByTime(-0.25, onethirtyfivedegree); //Turns Counter-clock-wise at one-quarter speed for .8 seconds to make 135-DEGREE (2 X 45-DEGREE) turn  correction  not sure why 90 is not enough
        // checking which side was Blue -  Left Side of Beacon 1 is furter away from Right Side and requires more time
        if (determinedSide == "left") {  //check side to determine time to reach next beacon ( left is longer than right)
            moveByTime(0.25, 2000);  //Move Forward  at one-quarter speed for 1.75 seconds
        } else if (determinedSide  == "right") {
            moveByTime(0.25, 1750);  //Move Forward  at one-quarter speed for 1.5 seconds
        }

        //moveByTime(-0.25, 50); // Move Backward at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward  at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)

        turnByTime(0.25, ninetydegrees); //Turns Clock-wise at one-quarter speed for .81 seconds to make 90-DEGREE turn (2 x 45-DEGREE turn)
        moveByTime(0.25, 500); //Move Forward  at one-quarter speed for .5 a seconds to get closer to beacon
        moveByTime(0.25, 250); //Move Forward  at one-quarter speed for .5 a seconds to get closer to beacon
        goForButton(); //Determines blue side of beacon and hits button on that side

        //telemetry.addData("Claim Blue Beacon 2 Done");
        //telemetry.update();

        //Follow same path as if to Claim cap Ball  ***assumption Cap Ball has already been claimed by alliance partner***

        moveByTime(-0.25, 1000); //Move Backwards at one-quarter speed for .5 second changed to 1 second
        turnByTime(0.25, onethirtyfivedegree); //Turns Clock-wise at half speed for 1.6 seconds to make 135-DEGREE turn (3 X 45-DEGREE)  to put intake in front position
        //moveByTime(-0.25, 50); // Move Backward  at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward   at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        moveByTime(0.25, 2500); //Move Forwards  at one-quarter speed for 1.5 seconds changed to 2.5 seconds for longer distance
        turnByTime(0.25, fourtyfivedegrees); //Turns Clock-wise at one-quarter speed for .4 to make 45-DEGREE turn
        moveByTime(0.25, 1500); //Move forwards at one-quarter speed for 1.5 seconds

        //Park at Blue Corner Vortex

        turnByTime(-0.25, ninetydegrees); //Turns counterClock-wise at one-quarter speed for three-quarters of a second to make 90-degree turn  ***was 45 degrees***
        moveByTime(0.25, 500); //Move  Forwards at one-quarter speed for one-half second
        moveByTime(0.25, 1000); //Move Forwards at one-quarter speed for one second
        stopAllMotors();
    }

    // Place all Method Code Below


    // Go For Button

    public void goForButton() throws InterruptedException {

        boolean dec = false;
        determinedSide = determineBlueSide();

        while (!dec) {

            if (determinedSide == "right") {

                moveByTime(-0.25, 350); //Move Backwards at one-quarter speed for .3.5 seconds to initial Beacon Position
                turnByTime(-0.25, fourtyfivedegrees); //Turns counter-clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 750); //Move forwards at one-quarter speed for quarter .25 second
                turnByTime(0.25, fourtyfivedegrees); //Turns clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 250); //Move forwards at one-quarter speed for quarter .25 second
                moveByTime(0.25, 50); //Move forwards at one-quarter speed for quarter .25 second
                dec = true;
            } else if (determinedSide == "left") {

                moveByTime(-0.25, 350); //Move Backwards at one-quarter speed for .3.5 seconds to initial Beacon Position
                turnByTime(0.25, fourtyfivedegrees); //Turns clock-wise at one-quarter speed for three-quarters of .41 seconds to make 45-degree turn
                moveByTime(0.25, 750); //Move forwards at one-quarter speed for .25 a seconds
                turnByTime(-0.25, fourtyfivedegrees); //Turns counter-clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 250); //Move forwards at one-quarter speed for .25 a seconds
                moveByTime(0.25, 50); //Move forwards at one-quarter speed for quarter .25 second
                dec = true;
            } else if (determinedSide != "left" && determinedSide != "right") { //  != represents "NOT EQUAL" || represents "or"
                moveByTime(0.25, 50);  //move forward at one-quarter speed for .125 seconds
                determinedSide = determineBlueSide(); // repeat
            }
        }
    }


    //Determine Blue Side

    public String determineBlueSide() throws InterruptedException {

        boolean dec = false;
        int c1 = 0; //Iterations counter
        String c = "";
        runtime2.reset();
        while (!dec) {
            if ((CSleft.red() * 8 > CSleft.blue() * 8) && (CSright.blue() * 8 > CSright.red() * 8)) {
                c = "right"; // If Left side of Beacon is Red and Right Side is Blue set direction to Right to push Blue button
                dec = true;
            } else if ((CSleft.blue() * 8 > CSleft.red() * 8) && (CSright.red() * 8 > CSright.blue() * 8)) {
                c = "left";  // If Left side of Beacon is Blue and Right Side is Red set direction to Left to push Blue button
                dec = true;
            } else if ((CSleft.red() * 8 > CSleft.blue() * 8)) {
                c = "right"; // If Left side of Beacon is Red  Set direction to Right to push Blue button
                dec = true;
            } else if ((CSleft.blue() * 8 > CSleft.red() * 8)) {
                c = "left";  // If Left side of Beacon is Blue set direction to Left to push Blue button
                dec = true;
            } else if ((CSright.blue() * 8 > CSright.red() * 8)) {
                c = "right"; // If Right Side is Blue set direction to Right to push Blue button
                dec = true;
            } else if ((CSright.red() * 8 > CSright.blue() * 8)) {
                c = "left";  // If Right Side is Red set direction to Left to push Blue button
                dec = true;
            } else if (runtime2.time() > 2000) {
                c = "null";  //if neither is true set direction to null
                //c = "right"; // Force to right to check code - for testing only
                dec = true;
            }
            c1++;

            telemetry.addData("LED", true ? "On" : "Off");
            telemetry.addData("L Red  ", CSleft.red() * 8);
            telemetry.addData("L Blue ", CSleft.blue() * 8);
            telemetry.addData("R Red  ", CSright.red() * 8);
            telemetry.addData("R Blue ", CSright.blue() * 8);
            telemetry.addData("Iterations: " + c1, null);
            telemetry.update();

        }

        return c;
    }

    // Move By Steps

    public void moveBySteps(double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        // (Class Variable : Array)

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        for (int i = 0; i < driveTrain.length; i++)
            driveTrain[i].setTargetPosition((int) (startPosition[i] + inches * COUNTS_PER_INCH));

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(Math.abs(power));
        BackLeft.setPower(Math.abs(power));
        FrontRight.setPower(Math.abs(power));
        BackRight.setPower(Math.abs(power));

        while (driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive())
            sleep(1);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Move By Time

    public void moveByTime(double power, int time) throws InterruptedException {

        for (DcMotor motor : driveTrain)  // Indexes through the drivetrain(array of motors)
            motor.setPower(power); //and sets each motor to the power

        sleep(time);  // basically idles the program so it will continue to do previous funtion.

        for (DcMotor motor : driveTrain)
            motor.setPower(0); // Resets motor power to 0 (stop) after executing move for defined
    }


    // Turn By Time

    public void turnByTime(double power, int time) throws InterruptedException {
        //Positive power makes robot turn right
        //Negative power makes robot turn left

        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);

        sleep(time);

        for (DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    // Shoot

    public void shoot(double power, double targetTime, double catapultDelay) throws InterruptedException {

        runtime1.reset();
        while (runtime1.time() < targetTime) {  //start shooter motors
            stopDriveTrain();
            r.setPower(power); //Right shooter wheel
            l.setPower(power); //Left Shooter wheel

            if (runtime1.time() > catapultDelay)   //Check if time to start catapult servo
                Catapult.setPower(power);  // set full power forward

            telemetry.addData("Time: " + runtime1.time(), null);
            telemetry.addData("Target Time: " + targetTime, null);
            telemetry.addData ("Catapult Delay" + catapultDelay,null);
            telemetry.update();
        }
        stopAllMotors();
        Catapult.setPower(catapultStop);  //Turn Catapult Servo motor off

    }

    // Stop All Motors

    public void stopAllMotors() throws InterruptedException {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        r.setPower(0);
        l.setPower(0);
        Catapult.setPower(.5);  //Turn Catapult Servo motor off

    }

    // Stop Drive Train

    public void stopDriveTrain() throws InterruptedException {

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

}


