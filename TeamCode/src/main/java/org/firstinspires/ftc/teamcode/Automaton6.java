package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

public class Automaton6 {

    DcMotor FrontRight = null;
    DcMotor FrontLeft = null;
    DcMotor BackRight = null;
    DcMotor BackLeft = null;

    DcMotor[] driveTrain = {FrontRight, FrontLeft, BackRight, BackLeft};

    Servo sensorArm = null;

    //IMU setup
    public final int NAVX_DIM_I2C_PORT = 0;
    public AHRS navx_device;
    public navXPIDController yawPIDController;

    public final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public final double TOLERANCE_DEGREES = 1.0;
    public final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    public final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    public final double YAW_PID_P = 0.005;
    public final double YAW_PID_I = 0.0;
    public final double YAW_PID_D = 0.0;

    //encoder constants
    static final double TAU                  = 6.283185;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES  = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);

    static final double DEGREES_TO_ENCODER_INCHES = 0;

    HardwareMap hwMap = null;

    public Automaton6() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        FrontRight = hwMap.dcMotor.get("FrontRight");
        FrontLeft = hwMap.dcMotor.get("FrontLeft");
        BackRight = hwMap.dcMotor.get("BackRight");
        BackLeft = hwMap.dcMotor.get("BackLeft");

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        sensorArm = hwMap.servo.get("sensorArm");
        sensorArm.setPosition(.5);

        navx_device = AHRS.getInstance(hwMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
    }
}