package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeCamera;
import org.firstinspires.ftc.robotcontroller.internal.testcode.MatrixControllerDemo;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
@Autonomous(name="Auto6322Red", group="Autonomous")
//@Disabled

public class Auto6322Red extends LinearOpModeCamera {

    String color = "";

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

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    int bnum = 0;
    int ds2 = 2;  // additional downsampling of the image

    //IMU setup
    AHRS navx_device;
    navXPIDController yawPIDController;

    final int NAVX_DIM_I2C_PORT = 5;

    final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    final double TOLERANCE_DEGREES = 1.0;
    final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    final double YAW_PID_P = 0.005;
    final double YAW_PID_I = 0.0;
    final double YAW_PID_D = 0.0;

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

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController(navx_device, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

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

        // wait for the start button to be pressed.
        waitForStart();

        driveStraight(0.5, 10);
        moveByTime(0.0, 500);
        //shoot(0.7, 5, 2);

        turnBySteps(0.5, 3.48);
        moveByTime(0.0, 500);

        runUntilWhite(0.3);
        moveByTime(0.0, 500);

        turnBySteps(0.5, -2.15);
        moveByTime(0.0, 500);



        //moveBySteps(0.5, -11);
        //moveByTime(0.0, 500);

        //turnBySteps(0.5, 3);

        //driveStraight(0.3, 6);

        /*turnBySteps(0.4, 3);
        moveByTime(0.0, 1000);
        turnBySteps(0.8, 5);
        driveStraight(0.4, 12);
        for (DcMotor motor : driveTrain)
            motor.setPower(0);

        if (determineColor() == "red") {
            moveBySteps(0.3, -3.5);
            rightPusher.setPower(-1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
            rightPusher.setPower(1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
        }
        else if (determineColor() == "blue") {
            moveBySteps(0.3, -9);
            rightPusher.setPower(-1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
            rightPusher.setPower(1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
        }

        moveBySteps(0.3, -6);
        runUntilWhite(0.3);
        adjustAtWhite();

        moveBySteps(0.3, 12);
        runUntilWhite(0.3);
        moveBySteps(0.3, -5);
        if (determineColor() == "red") {
            moveBySteps(0.3, -3.5);
            rightPusher.setPower(-1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
            rightPusher.setPower(1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
        }
        else if (determineColor() == "blue") {
            moveBySteps(0.3, -9);
            rightPusher.setPower(-1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
            rightPusher.setPower(1.0);
            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            sleep(1500);
        }
        runUntilWhite(0.3);
        turnBySteps(0.8, 12);
        moveBySteps(0.3, 6);
        turnBySteps(0.1, 25);
        conveyor.setPower(1.0);
        shooter.setPower(1.0);
        for (DcMotor motor: driveTrain)
            motor.setPower(0);
        sleep(2500);*/

        /*moveBySteps(0.5, 38);
        turnBySteps(0.2, -14);
        moveByTime(0, 1000);
        moveBySteps(0.3, 6);

        color = this.determineColor();
        if (color == "blue"){
            moveBySteps(0.5, 8);
        }
        else if (color == "red"){
            moveBySteps(0.5, 3);
        }
        else if (color == "null") {
            intake.setPower(0.5);
            sleep(1000);
        }
        leftPusher.setPower(-1.0);
        runtime1.reset();
        while (runtime1.time() < 1.5);
        leftPusher.setPower(0);*/

        //moveBySteps(0.2, 12);

        /*moveUntil(0.05, "red");
        moveByTime(0.0, 500);
        moveBySteps(0.2, 4.5);
        moveByTime(0.0, 50);
        leftPusher.setPower(-1.0);
        runtime1.reset();
        while (runtime1.time() < 1.5); //run while timer is less than 1.5 seconds
        leftPusher.setPower(0);
        sleep(1000);
        leftPusher.setPower(1.0);
        runtime1.reset();
        while (runtime1.time() < 1.5);
        leftPusher.setPower(0);
        sleep(1000);
        turnBySteps(0.6, 26);
        moveBySteps(0.6, 28);
        sleep(1500);
        turnBySteps(0.5, 18);
        moveBySteps(0.6, 24);*/

        //Starts autonomous using camera
        /*if (isCameraAvailable()) {

            setCameraDownsampling(8);
            // parameter determines how downsampled you want your images
            // 8, 4, 2, or 1.
            // higher number is more downsampled, so less resolution but faster
            // 1 is original resolution, which is detailed but slow
            // must be called before super.init sets up the camera

            startCamera();  // can take a while.
                            // best started before waitForStart
                            // or in a separate thread.

            waitForStart();

            stopCameraInSecs(30);   // set independent thread to kill the camera
                                    // when the mode is done
                                    // use 30 for auto, 120 for teleop

            /*moveBySteps(0.75, 6);
            turnBySteps(0.75, -8);
            moveBySteps(0.75, 62);
            turnBySteps(0.1, 6);*/

            /*while(ODSleft.getRawLightDetected() < 4.9 && ODSright.getRawLightDetected() < 4.9){
                FrontLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackLeft.setPower(0.5);
                BackRight.setPower(0.5);
            }
            int c = 0;

            stopCamera();
        }*/

    }

    public void runUntilWhite(double power) throws InterruptedException {
        boolean dec = false;
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;
        while (!dec) {
            if (ODSleft.getRawLightDetected()*13 < .8 || ODSright.getRawLightDetected() < .9) {
                for (DcMotor motor : driveTrain)
                leftSpeed = power;
                rightSpeed = power;

                leftSpeed = Range.clip(leftSpeed , -1.0, 1.0);
                rightSpeed = Range.clip(rightSpeed , -1.0, 1.0);

                FrontLeft.setPower(leftSpeed);
                BackLeft.setPower(leftSpeed);
                FrontRight.setPower(rightSpeed);
                BackRight.setPower(rightSpeed);
                idle();

            }
            else if (ODSleft.getRawLightDetected()*13 > .8 || ODSright.getRawLightDetected() > .9) {
                for (DcMotor motor : driveTrain)
                    motor.setPower(0);
                dec = true;
            }
            telemetry.addData("ODSleft Values: " + ODSleft.getRawLightDetected(), null);
            telemetry.addData("ODSright Values: " + ODSright.getRawLightDetected(), null);
            telemetry.update();
            sleep(1);
        }
    }

    public void shoot(double power, double time, double conveyorDelay) throws InterruptedException {
        runtime3.reset();
        while(runtime3.time() < time) {
            stopDriveTrain();
            shooter.setPower(power);

            if ((runtime3.time() > conveyorDelay) && (runtime3.time() < (conveyorDelay + 0.5))) {
                conveyor.setPower(1.0);
                intake.setPower(1.0);
            }
            else if (runtime3.time() > (conveyorDelay + 0.5) && runtime3.time() < (conveyorDelay + 1.0)) {
                intake.setPower(0.0);
                conveyor.setPower(0.0);
            }
            else if (runtime3.time() > (conveyorDelay + 1.5)) {
                conveyor.setPower(1.0);
                intake.setPower(1.0);
            }

        }
        shooter.setPower(0.0);
        conveyor.setPower(0.0);
        intake.setPower(0.0);

    }

    public void adjustAtWhite() throws InterruptedException {
        boolean dec = false;
        while (!dec) {
            if ((ODSright.getRawLightDetected() - ODSleft.getRawLightDetected()) > 0.02) {
                FrontLeft.setPower(0.1);
                BackLeft.setPower(0.1);
            }
            else if ((ODSleft.getRawLightDetected() - ODSright.getRawLightDetected()) > 0.02) {
                FrontRight.setPower(0.1);
                FrontRight.setPower(0.1);
            }
            else
                dec = true;
        }
    }

    public void turnByGyro(double power, int degrees) throws InterruptedException {

        int s = -1;

        // if the A and B buttons are pressed just now, reset Z heading.
        curResetState = true;
        if(curResetState && !lastResetState)  {
            gyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;

        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        heading = gyro.getHeading();
        angleZ  = gyro.getIntegratedZValue();

        double target = gyro.getHeading() + degrees;

        while((Math.abs((target - gyro.getHeading()))) > 0) {
            for (DcMotor motor : driveTrain) {
                motor.setPower(power * s);
                s *= -1;
            }
            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);
            telemetry.update();
            telemetry.update();
            waitOneFullHardwareCycle();
        }

        for (DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    public void driveStraight(double power, double inches) throws InterruptedException{

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

    public String determineColor() throws InterruptedException {

        boolean dec = false;
        int c1 = 0; //Iterations counter
        String c = "";

        while(!dec) {
            if (CSleft.red() * 8 > CSleft.blue() * 8) {
                c = "red";
                dec = true;
            }
            else if (CSleft.blue() * 8 > CSleft.red() * 8) {
                c = "blue";
                dec = true;
            }
            else if (c1 > 500000) {
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

    public void moveUntil(double power, String color) throws InterruptedException {

        boolean dec = false;
        int c = 0;

        float hsvValues[] = {0F, 0F, 0F};

        //telemetry.addData("Iterations " + ++c, null);

        for (DcMotor motor : driveTrain)
            motor.setPower(power);

        if (color.equals("white")) {
            while (!dec) {
                if (CSleft.red() > 8 && CSleft.green() > 8 && CSleft.blue() > 8)
                    dec = true;
                telemetry.addData("LED", true ? "On" : "Off");
                telemetry.addData("Red  ", CSleft.red() * 8);
                telemetry.addData("Blue ", CSleft.blue() * 8);
                telemetry.update();
            }
        }

        if (color.equals("red")) {
            runtime2.reset();
            while (!dec) {
                if (((CSleft.red() * 8) > (CSleft.blue() * 8)) || ((CSleft.red() * 8) > 4))
                    dec = true;
                else if (runtime2.time() >  7)
                    dec = true;
                telemetry.addData("LED", true ? "On" : "Off");
                telemetry.addData("Red  ", CSleft.red()*8);
                telemetry.addData("Blue ", CSleft.blue()*8);
                telemetry.update();
            }
        }

        if (color.equals("blue")) {
            while (!dec) {
                if (((CSleft.blue() * 8) > (CSleft.red() * 8)) || ((CSleft.blue() * 8) > 4))
                    dec = true;
                telemetry.addData("LED", true ? "On" : "Off");
                telemetry.addData("Red  ", CSleft.red()*8);
                telemetry.addData("Blue ", CSleft.blue()*8);
                telemetry.update();
            }
        }

        for (DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    public void stopDriveTrain() throws InterruptedException {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void moveBySteps(double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

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

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newLeftTarget);
            FrontRight.setTargetPosition(newRightTarget);
            newLeftTarget = BackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = BackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            BackLeft.setTargetPosition(newLeftTarget);
            BackRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime1.reset();
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);

            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime1.reset();
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime1.seconds() < timeoutS) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());

                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1000);
        }
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

    public void turnSideBySteps(String side, double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        //if (side == "right")

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

    //Uses gyroscopic features in the NAVX Micro Sensor
    public void turnByNavx(double power, double angle) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        boolean turnComplete = false;

        navx_device.zeroYaw(); //Resets yaw to zero
        yawPIDController.setSetpoint(angle); //Sets desired angle

        int startPosition;
        double neededInches = angle * DEGREES_TO_ENCODER_INCHES;

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        startPosition = FrontLeft.getCurrentPosition();

        try {
            yawPIDController.enable(true);

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;

            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) && !Thread.currentThread().isInterrupted() && !turnComplete) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {

                    if (yawPIDResult.isOnTarget()) {

                        for (DcMotor motor : driveTrain)
                            motor.setPower(0);

                        turnComplete = true;
                    }
                    else {

                        double output = yawPIDResult.getOutput();

                        FrontRight.setPower(-output);
                        FrontLeft.setPower(output);
                        BackRight.setPower(-output);
                        BackLeft.setPower(output);

                        telemetry.addData("PIDOutput", df.format(output) + ", " + df.format(-output));
                    }
                }
                else {
                    // A timeout occurred
                    telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    turnBySteps(power, (neededInches + startPosition) - FrontLeft.getCurrentPosition());
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            }
        }
        catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    //Determines the color of the button using the ZTE Speed Camera
    /*public int determineButton() {

        int benum = 0;

        int color;

        String colorString = "";

        int redValue;
        int blueValue;
        int greenValue;

        if (imageReady()) {

            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

            int pixel = rgbImage.getPixel(width/2/ds2, height/2/ds2);
            redValue = red(pixel);
            blueValue = blue(pixel);
            greenValue = green(pixel);

            color = highestColor(redValue, greenValue, blueValue);

            switch (color) {
                    case 0:
                        colorString = "RED";
                        break;
                    case 1:
                        colorString = "GREEN";
                        break;
                    case 2:
                        colorString = "BLUE";
            }

            telemetry.addData("Color:", "highest color: " + colorString);
            telemetry.addData("Color:", "Red value: " + redValue);
            telemetry.addData("Color:", "Green value: " + greenValue);
            telemetry.addData("Color:", "Blue value: " + blueValue);

            telemetry.update();

            if (colorString.equals("BLUE"))
                benum = 1;
            else if (colorString.equals("RED"))
                benum = 2;

        }

        return benum;
    }*/

}