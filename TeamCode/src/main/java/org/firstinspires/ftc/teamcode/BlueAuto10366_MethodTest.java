package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.PublicKey;
/**
 * Created by Carol on 1/15/2017.
 */
// Comment to pus file

@Autonomous(name = "BlueAuto10366_MethodTest", group = "Autonomous")
    public class BlueAuto10366_MethodTest extends LinearOpMode{  // LinearOpMode is for autonomous

    // @TeleOp(name="BlueAuto10366_MethodTest", group ="Opmode")  for TeleOp
    // public class BlueAuto10366_MethodTest extends Linear{  // Linear is for TeleOp

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


                //Intake Motor Declaration

    DcMotor intake;


                //Lift Motor Declaration

    DcMotor lift;


                //Color Sensor Declarations

    ColorSensor CSleft;
    ColorSensor CSright;



            //Encoder Constants

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
        //Catapult.setDirection(CRServo.Direction.REVERSE); //would need to use if servo was mounted backwards
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

                    // /Shooting 2 Balls Good distance  Manual Shooting perfect.  Need to get Servo Working

        Catapult.setPower(.5); //Sets catapult servo to stop

        shoot(1.0, 3.95, 1.25); //Shoots particles at full power for 4.05 (3 full rotations)seconds and starts catapult after 1 seconds
        //shoot(1.0, 2.5, 0.3); //Shoots particles at full power for 2 seconds and set catapult Delay to 0 to test catapult
    }


    public void shoot(double power, double targetTime, double catapultDelay) throws InterruptedException {

        runtime1.reset();
        while (runtime1.time() < targetTime) {  //start shooter motors
            stopDriveTrain();
         r.setPower(power); //Right shooter wheel
         l.setPower(power); //Left Shooter wheel
            if (runtime1.time() > catapultDelay) {  //Check if time to start catapult servo
                Catapult.setPower(power);  // set full power forward
            }
            telemetry.addData("Time: " + runtime1.time(), null);
            telemetry.addData("Target Time: " + targetTime, null);
            telemetry.update();
        }
        stopAllMotors();

    }

    public void stopDriveTrain() throws InterruptedException {

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void stopAllMotors() throws InterruptedException {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        r.setPower(0);
        l.setPower(0);
        Catapult.setPower(.5);  //Turn Catapult Servo motor off
    }

}

