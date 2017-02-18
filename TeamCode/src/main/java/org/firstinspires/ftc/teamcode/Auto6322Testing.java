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


import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeCamera;

/**
 * Created by Dylanjamaludin on 2/16/17.
 */
@Autonomous(name="testing", group="Autonomous")

public class Auto6322Testing extends LinearOpMode{

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

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController(navx_device, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        // wait for the start button to be pressed.
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        //yawPIDController.setTolerance(TimestampedPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        waitForStart();

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }

        turnByAngle(90); //90 degrees

        navx_device.close();
        telemetry.addData("LinearOp", "Complete");
    }

    //Uses gyroscopic features in the NAVX Micro Sensor
    public void turnByAngle(double angle) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        boolean turnComplete = false;

        navx_device.zeroYaw(); //Resets yaw to zero
        yawPIDController.setSetpoint(angle); //Sets desired angle

        final double TOTAL_RUN_TIME_SECONDS = 30.0;

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) && !Thread.currentThread().isInterrupted() && !turnComplete) {

            double output = yawPIDResult.getOutput();

            if (!yawPIDResult.isOnTarget()) {

                FrontRight.setPower(-output);
                FrontLeft.setPower(output);
                BackRight.setPower(-output);
                BackLeft.setPower(output);

                //telemetry.addData("PIDOutput", df.format(output) + ", " + df.format(-output));
            }
            else {

                for (DcMotor motor : driveTrain)
                    motor.setPower(0);

                turnComplete = true;
            }

            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            telemetry.addData("yawPIDResult" + yawPIDResult.getOutput(), null);
            telemetry.update();
        }
        telemetry.addData("DONE!!!!!", null);

    }

}
