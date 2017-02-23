package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import com.kauailabs.navx.ftc.*;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.Range;


import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeCamera;

/**
 * Created by Dylanjamaludin on 2/16/17.
 */
@Autonomous(name="Auto6322Testing", group="Autonomous")

public class Auto6322Testing extends Auto6322Red{

    String color = "";

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime runtime1 = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();
    ElapsedTime runtime3 = new ElapsedTime();

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

    //Drive Train Motor Declarations
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    final DcMotor[] driveTrain = new DcMotor[4];
    final DcMotor[] rightDriveTrain = new DcMotor[2];
    final DcMotor[] leftDriveTrain = new DcMotor[2];

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

    //Shooting Mechanism Motor Declarations
    DcMotor shooter;

    //Intake Motor Declaration
    DcMotor intake;

    //Conveyor Belt Motor Declaration
    DcMotor conveyor;

    //Linear Slide Motor Declaration
    DcMotor linear;

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

    //Continuous Rotation Servo Declarations
    CRServo rightPusher;
    CRServo leftPusher;

    //Locking mechanism for cap ball lifter
    Servo lock;

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    //Optical Distance Sensor Declaration
    OpticalDistanceSensor ODSleft;
    OpticalDistanceSensor ODSright;

    //Gyro Sensor
    ModernRoboticsI2cGyro gyro;

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

    boolean calibration_complete = false;

    int bnum = 0;
    int ds2 = 2;  // additional downsampling of the image

    //IMU setup
    AHRS navx_device;
    navXPIDController yawPIDController;

    final int NAVX_DIM_I2C_PORT = 5;

    final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    final double TOLERANCE_DEGREES = 2.0;
    final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    final double YAW_PID_P = 0.005;
    final double YAW_PID_I = 0.0;
    final double YAW_PID_D = 0.0;

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    //encoder constants
    static final double TAU                  = 6.283185;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES  = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);
    static final double DEGREES_TO_ENCODER_INCHES = 0;

    @Override
    public void runOpMode() throws InterruptedException {

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        //Drive Train Motors
        FrontRight = hardwareMap.dcMotor.get("fr");
        FrontLeft = hardwareMap.dcMotor.get("fl");
        BackRight = hardwareMap.dcMotor.get("br");
        BackLeft = hardwareMap.dcMotor.get("bl");

        driveTrain[0] = FrontRight;
        driveTrain[1] = FrontLeft;
        driveTrain[2] = BackRight;
        driveTrain[3] = BackLeft;
        rightDriveTrain[0] = FrontRight;
        rightDriveTrain[1] = BackRight;
        leftDriveTrain[0] = FrontLeft;
        leftDriveTrain[1] = BackLeft;

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        //Shooting Mechanism Motors
        shooter = hardwareMap.dcMotor.get("s");

        //Linear Slide Motor Assignment
        linear = hardwareMap.dcMotor.get("w");

        //Intake Motor(s)
        intake = hardwareMap.dcMotor.get("i");

        //Conveyor Mechanism Motor
        conveyor = hardwareMap.dcMotor.get("c");

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        //Continuous Rotation Sensors
        rightPusher = hardwareMap.crservo.get("rp");
        leftPusher = hardwareMap.crservo.get("lp");

        //Lock
        lock = hardwareMap.servo.get("k");

        //Lock Mechanism Function
        lock.setPosition(0);

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        //Gyro Sensor Assignment
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("g");

        //Color Sensors
        CSleft = hardwareMap.colorSensor.get("csl");
        CSright = hardwareMap.colorSensor.get("csr");

        CSright.enableLed(true);
        CSleft.enableLed(true);
        CSleft.enableLed(false);
        CSright.enableLed(false);

        //Optical Distance Sensors
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController(navx_device, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setContinuous(false);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();//START HERE

        driveStraight(0.5, 1);
        moveByTime(0.0, 500);
        //shoot(0.7, 5, 2);

        turnBySteps(0.5, 5.2);
        moveByTime(0.0, 500);


        driveStraight(0.5, 32);
        moveByTime(0.0, 500);

        turnBySteps(0.5, -3.2);
        //moveByTime(0.0, 500);

        //moveBySteps(0.5, -11);
        //moveByTime(0.0, 500);

        //turnBySteps(0.5, 3);

        //driveStraight(0.3, 6);

        // testing method turnbyangle2
       // turnbyangle2(0.5,90,1);


        /*while ( !calibration_complete ) {
            *//* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             *//*
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
            calibration_complete = !navx_device.isCalibrating();
        }

        turnByNavx(90); //90 degrees

        navx_device.close();
        telemetry.addData("LinearOp", "Complete");*/

        //driveStraight(0.3, 10);
        //moveByTime(0.0, 1000);
        //shoot(1.0, 5, 2); //Power, time, conveyordelay

        //turnBySteps(0.4, 10);
        //turnByGyro(0.5, 48);
        //moveByTime(0.0, 3000);
    }

    //Uses gyroscopic features in the NAVX Micro Sensor
    public void turnByNavx(double angle) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        boolean turnComplete = false;
        final double TOTAL_RUN_TIME_SECONDS = 30.0;

        navx_device.zeroYaw(); //Resets yaw to zero
        yawPIDController.setSetpoint(angle); //Sets desired angle

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) && !Thread.currentThread().isInterrupted()) {

            double output = yawPIDResult.getOutput();
            int DEVICE_TIMEOUT_MS = 500;

            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (!yawPIDResult.isOnTarget()) {

                    FrontRight.setPower(-output);
                    FrontLeft.setPower(output);
                    BackRight.setPower(-output);
                    BackLeft.setPower(output);

                    //telemetry.addData("PIDOutput", df.format(output) + ", " + df.format(-output));
                } else {

                    for (DcMotor motor : driveTrain)
                        motor.setPower(0);

                    turnComplete = true;
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.addData("Output: " + yawPIDResult.getOutput(), null);
                telemetry.update();
            }

            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            telemetry.addData("Output: " + yawPIDResult.getOutput(), null);
            telemetry.update();
        }
        telemetry.addData("DONE!!!!!", null);

    }

    public void turnByGyro(double power, int degrees) throws InterruptedException {

        double constantOfDegrees = (2/3);

        int s = -1;
        boolean turnComplete = false;
        double initialPosition = gyro.getIntegratedZValue();
        gyro.resetZAxisIntegrator();

        while (!turnComplete) {


            double currentPosition = gyro.getIntegratedZValue();
            double target = initialPosition + (degrees - 45);

            if ((Math.abs(target)) > currentPosition) {
                for (DcMotor motor : driveTrain) {
                    motor.setPower(power * s);
                    s *= -1;
                }
            }
            else
                turnComplete = true;
            telemetry.addData("Degrees: " + currentPosition, null);
            telemetry.update();
        }

        for (DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    public void driveStraight(double power, int inches) throws InterruptedException{

        double leftSpeed;
        double rightSpeed;

        double target = gyro.getHeading();
        double startPosition = FrontLeft.getCurrentPosition();

        while(FrontLeft.getCurrentPosition() < (inches*COUNTS_PER_INCH) + startPosition){

            int zAccumulated = gyro.getHeading();

            leftSpeed = power + (zAccumulated - target)/100;
            rightSpeed = power + (zAccumulated - target)/100;

            leftSpeed = Range.clip(leftSpeed, -1.0, 1.0);
            rightSpeed = Range.clip(rightSpeed , -1.0, 1.0);

            FrontLeft.setPower(leftSpeed);
            BackLeft.setPower(leftSpeed);
            FrontRight.setPower(rightSpeed);
            BackRight.setPower(rightSpeed);
            idle();

        }
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

    }

    public void moveBySteps(double power, double inches) throws InterruptedException {

        int o = 0;
        int b = 10;
        double c = 0.5;
        double p = 0.0;
        int t = 0;
        int[] startPosition = new int[4];

        if(t == 0){
            b = 11;
            for (double i = 0; i < 10; i++){

                b = b - 1;
                p = Math.pow(c,b);
                t++;

            }
        }

        if (t == 10){
            b = 0 ;
            for (int i = 0; i < 10; i++){

                b = b + 1;
                p = Math.pow(c,b);

            }
        }

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        shooter.setPower(p);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        for (int i = 0; i < driveTrain.length; i++)
            driveTrain[i].setTargetPosition((int)(startPosition[i] + inches * COUNTS_PER_INCH));

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : driveTrain)
            motor.setPower(Math.abs(power));

        while(driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive())
            sleep(1);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnBySteps(double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        FrontRight.setTargetPosition((int)(startPosition[0] + -inches * COUNTS_PER_INCH));
        FrontLeft.setTargetPosition((int)(startPosition[1] + inches * COUNTS_PER_INCH));
        BackRight.setTargetPosition((int)(startPosition[2] + -inches * COUNTS_PER_INCH));
        BackLeft.setTargetPosition((int)(startPosition[3] + inches * COUNTS_PER_INCH));

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : driveTrain)
            motor.setPower(Math.abs(power));

        while(driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive())
            sleep(1);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void moveByTime(double power, int time) throws InterruptedException {

        for(DcMotor motor : driveTrain)
            motor.setPower(power);

        sleep(time);

        for(DcMotor motor : driveTrain)
            motor.setPower(0);
    }
    public void turnbyangle2 (double power, double targetHeading, int right_Left) {
        //double constantOfDegrees = (2 / 3);

        int s;
        boolean turnComplete = false;
        double initialPosition = gyro.getHeading();


        if (targetHeading ==360) {
            targetHeading = 0;
        }

        if (right_Left == 1) {
        while (!turnComplete) {


            double currentPosition = gyro.getHeading();
            double target = initialPosition + (targetHeading);
            target = 360 - target;
            s = -1;
            if ((Math.abs(target)) > currentPosition) {
                for (DcMotor motor : driveTrain) {
                    motor.setPower(power * s);
                    s *= -1;
                }
            } else
                turnComplete = true;
            telemetry.addData("Degrees: " + currentPosition, null);
            telemetry.update();
        }

        for (DcMotor motor : driveTrain)
            motor.setPower(0);
        }
        else if (right_Left == 0){
            while (!turnComplete){

                double currentPosition = gyro.getHeading();
                double target = initialPosition + (targetHeading);
                s = 1;

                if ((Math.abs(target)) > currentPosition) {
                    for (DcMotor motor : driveTrain) {
                        motor.setPower(power * s);
                        s *= -1;
                    }
                } else
                    turnComplete = true;
                telemetry.addData("Degrees: " + currentPosition, null);
                telemetry.update();
            }

            for (DcMotor motor : driveTrain)
                motor.setPower(0);
        }

    }


}

