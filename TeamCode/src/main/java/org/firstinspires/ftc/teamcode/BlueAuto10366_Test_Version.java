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
 * Created by SethHorwitz on 12/17/16.
 */
@Autonomous(name="BlueAuto1_10366_Test", group="Autonomous")

public class BlueAuto10366_Test_Version extends LinearOpMode {

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
    CRServo Catapult;  //Continuous Rotation Servo
    Servo Left;  // Left lift fork locking servo
    Servo Right;  // Right lift fork locking sevro

    //Intake Motor Declaration
    DcMotor intake;

    //Lift Motor Declaration
    DcMotor lift;

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    // Misc code for Camera ZTE Speed
    int bnum = 0;
    int ds2 = 2;  // additional downsampling of the image

    float hsvValues[] = {0F, 0F, 0F};

    //encoder constants

    //Moror Values from Andymark for NeverRest 60
    static final double TAU = 6.283185;  // = 2*pi
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);
    static final double DEGREES_TO_ENCODER_INCHES = 0;

    //Moror Values from Andymark for NeverRest 20
    static final double TAU_20 = 6.283185;
    static final double COUNTS_PER_MOTOR_REV_20 = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION_20 = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES_20 = 2.0;     // For figuring circumference
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
        //Catapult.setDirection(CRServo.Direction.REVERSE);
        Right.setDirection(Servo.Direction.REVERSE); // sets right lift fork servo to go opposite of left

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

        CSright.enableLed(true);  // need to set on (true) and off(false) to get sensors to function correctly
        CSleft.enableLed(true);
        CSleft.enableLed(false);
        CSright.enableLed(false);

        waitForStart(); //Autonomous begins when play button is pressed on the Driver Station Phone

        moveByTime(-0.25, 10);  //Move Backward  at one-quarter speed for  .010 seconds  ***code to correct initial counter-clock-wise turn
        turnByTime(-0.25, 25); // Move Forward  at half speed for  .015 seconds counter clocl-wise ***code to correct initial clock-wise turn
        moveByTime(-0.25, 1250); //move Backward at one-quarter speed for 1.250 seconds

        // /Shooting 2 Balls Good distance  Manual Shooting perfect.  Need to get Servo Working
        Catapult.setPower(.5); //Sets catapult servo to stop


        shoot(1.0, 500, 125); //Shoots particles at full power for 1 seconds and starts catapult after .25 seconds

        //Claim Blue Beacon 1

        moveByTime(0.25, 850); //Move Forward at one quarter speed for .600 changed to ***.750 to 850 after ball was inflated***
        turnByTime(0.25, 405); //Turn Clock-wise at one-quarter speed for .410 seconds(.425 seconds - .15 Seconds) to offset (initialization) to make 45-degree turn
        moveByTime(-0.25, 1750); //Move Backward at one-quarter speed for 1.500 seconds ***Battery Full charge  14.44 - 14.00**
        turnByTime(-0.25, 1215); //Turns Counter-clock-wise at one-quarter speed for 1.230 seconds to make 135-degree turn
        //moveByTime(-0.25, 50); // Move Backward  at one-quarter speed for .050 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward   at one-quarter speed for .050 seconds (***used to correct motor direction to go straight***)
        moveByTime(0.25, 500); // Move Forward at one-quarter speed for .650 seconds to establish initial Beacon Startion position
        moveByTime(0.25, 500); // Move Forward at one-quarter speed for .350 seconds to get closer to beacon
        goForButton(); //Determines blue side of beacon and hits button on that side


        //Claim Blue Beacon 2
        //moveByTime(-0.25, 500); //Move Backwards at half speed for .5 seconds
        //turnByTime(-0.25, 800); //Turns Counter-clock-wise at one-quarter speed for .8 seconds to make 90-degree turn
        //moveByTime(-0.25, 50); // Move Backward at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward  at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 1250); //Move Forward  at one-quarter speed for 1.25 seconds
        //turnByTime(0.25, 800); //Turns Clock-wise at one-quarter speed for .8 seconds to make 90-degree turn
        //moveByTime(0.25, 500); //Move Forward  at one-quarter speed for .5 a seconds to get closer to beacon
        //goForButton(); //Determines blue side of beacon and hits button on that side

        //Claim Blue Cap Ball
        //moveByTime(-0.25, 500); //Move Backwards at one-quarter speed for .5 second
        //turnByTime(-0.25, 1600); //Turns Counter-clock-wise at half speed for 1.6 seconds to make 135-degree turn
        //moveByTime(-0.25, 50); // Move Backward  at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 50);  // Move Forward   at one-quarter speed for .05 seconds (***used to correct motor direction to go straight***)
        //moveByTime(0.25, 1500); //Move Forwards  at one-quarter speed for 1.5 seconds
        //turnByTime(0.25, 400); //Turns Clock-wise at one-quarter speed for .4 to make 45-degree turn
        //moveByTime(0.25, 1500); //Move forwards at one-quarter speed for 1.5 seconds

        //Park at Blue Corner Vortex
        //turnByTime(0.25, 400); //Turns Clock-wise at one-quarter speed for three-quarters of a second to make 45-degree turn
        //moveByTime(0.25, 500); //Move  Forwards at one-quarter speed for one-half second
        //moveByTime(0.25, 1000); //Move Forwards at one-quarter speed for one second
        stopAllMotors();
    }

    // Place all Method Code Below
    public void goForButton() throws InterruptedException {

        boolean dec = false;
        String determinedSide;
        determinedSide = determineBlueSide();

        while (!dec) {

            if (determinedSide == "left") {

                moveByTime(-0.25, 350);  //Move Backwards at one-quarter speed for .3.5 seconds to initial Beacon Position
                turnByTime(-0.25, 410); //Turns counter-clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for quarter .25 second
                turnByTime(0.25, 410); //Turns clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 250); //Move forwards at one-quarter speed for quarter .25 second
                moveByTime(0.25, 100); //Move forwards at one-quarter speed for quarter .25 second
                dec = true;
            } else if (determinedSide == "right") {

                moveByTime(-0.25, 350); //Move Backwards at one-quarter speed for .3.5 seconds to initial Beacon Position
                turnByTime(0.25, 410); //Turns clock-wise at one-quarter speed for three-quarters of .41 seconds to make 45-degree turn
                moveByTime(0.25, 500); //Move forwards at one-quarter speed for .25 a seconds
                turnByTime(-0.25, 410); //Turns counter-clock-wise at one-quarter speed for three-quarters of .4 seconds to make 45-degree turn
                moveByTime(0.25, 250); //Move forwards at one-quarter speed for .25 a seconds
                moveByTime(0.25, 100); //Move forwards at one-quarter speed for quarter .25 second
                dec = true;
            } else if (determinedSide != "left" && determinedSide != "right") { //  != represents "NOT EQUAL" || represents "or"
                moveByTime(0.25, 125);  //move forward at one-quarter speed for .125 seconds
                determinedSide = determineBlueSide(); // repeat
            }
        }
    }

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

    public void moveByTime(double power, int time) throws InterruptedException {

        for (DcMotor motor : driveTrain)  // Indexes through the drivetrain(array of motors)
            motor.setPower(power); //and sets each motor to the power

        sleep(time);  // basically idles the program so it will continue to do previous funtion.

        for (DcMotor motor : driveTrain)
            motor.setPower(0); // Resets motor power to 0 (stop) after executing move for defined
    }

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

    public void shoot(double power, int targetTime, int catapultDelay) throws InterruptedException {

        runtime1.reset();
        while (runtime1.time() < targetTime) {  //start shooter motors
            stopDriveTrain();
            r.setPower(power); //Right shooter wheel
            l.setPower(power); //Left Shooter wheel
            //sleep(125);
            sleep(catapultDelay);
            if (runtime1.time() > catapultDelay) {  //Check if time to start catapult servo
                Catapult.setPower(1);  // set full power forward
                //sleep(750);  // continue for .75 seconds
                sleep(targetTime-catapultDelay+50);
            }
        }
        if (runtime1.time() > targetTime) {  // Check if finished
            r.setPower(0);  // Turn Right shooter motor off
            l.setPower(0);  // Turn LefT shooter motor off
            Catapult.setPower(.5);  //Turn Catapult Servo motor off
        }
    }

    public void stopAllMotors() throws InterruptedException {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        r.setPower(0);
        l.setPower(0);
    }

    public void stopDriveTrain() throws InterruptedException {

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

}
