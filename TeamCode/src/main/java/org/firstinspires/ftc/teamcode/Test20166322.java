package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
@Autonomous(name="6322AutoTest", group="planning")
//@Disabled

public class Test20166322 extends LinearOpModeCamera {

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;
    
    DcMotor[] driveTrain = {FrontRight, FrontLeft, BackRight, BackLeft};

    Servo sensorArm;

    ColorSensor colorSensor;

    int ds2 = 2;  // additional downsampling of the image

    //IMU setup
    final int NAVX_DIM_I2C_PORT = 0;
    AHRS navx_device;
    navXPIDController yawPIDController;

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
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES  = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);

    static final double DEGREES_TO_ENCODER_INCHES = 0;

    @Override
    public void runOpMode() throws InterruptedException {

    	FrontRight = hardwareMap.dcMotor.get("FrontRight");
    	FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
    	BackRight = hardwareMap.dcMotor.get("BackRight");
    	BackLeft = hardwareMap.dcMotor.get("BackLeft");

    	for (DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    	for (DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    	FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    	BackLeft.setDirection(DcMotor.Direction.REVERSE);

        sensorArm = hardwareMap.servo.get("sensorArm");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                      NAVX_DIM_I2C_PORT,
                      AHRS.DeviceDataType.kProcessedData,
                      NAVX_DEVICE_UPDATE_RATE_HZ);

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        if (isCameraAvailable()) {

            // setCameraDownsampling(8);
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

            while (opModeIsActive()) {

                if (imageReady()) { // only do this if an image has been returned from the camera
                    int redValue = 0;
                    int blueValue = 0;
                    int greenValue = 0;

                    Bitmap rgbImage;
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
                    for (int x = 0; x < width / ds2; x++) {
                        for (int y = 0; y < height / ds2; y++) {
                            int pixel = rgbImage.getPixel(x, y);
                            redValue += red(pixel);
                            blueValue += blue(pixel);
                            greenValue += green(pixel);
                        }
                    }
                    int color = highestColor(redValue, greenValue, blueValue);

                    //checks the center of the image only
                    //int pixel = rgbImage.getPixel(width/2/ds2, height/2/ds2);
                    //int color = highestColor(red(pixel), green(pixel), blue(pixel));

                }

                telemetry.addData("Color:", "Color detected is: ");
                telemetry.update();
                sleep(10);
            }

            stopCamera();
        }
    }

    public void moveByTime(double power, int time) throws InterruptedException {

		for(DcMotor motor : driveTrain)
    		motor.setPower(power);    	

    	sleep(time);

		for(DcMotor motor : driveTrain)
    		motor.setPower(0);
    }

    public void moveBySteps(double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        for(DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for(int i = 0; i < driveTrain.length; i++)
            startPosition[i] = driveTrain[i].getCurrentPosition();

        for(int i = 0; i < driveTrain.length; i++)
            driveTrain[i].setTargetPosition((int)(startPosition[i] + inches * COUNTS_PER_INCH));

        for(DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for(DcMotor motor : driveTrain)
            motor.setPower(Math.abs(power));

        while(driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive()) {idle();}

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUntil(double power, String color) throws InterruptedException {

        boolean dec = false;

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        colorSensor.enableLed(true);

        Color.RGBToHSV(colorSensor.red( * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        for(DcMotor motor : driveTrain)
            motor.setPower(power);

        if (color.equals("white")) {
            while (!dec) {
                if (colorSensor.red() > 10 && colorSensor.green() > 10 && colorSensor.blue() > 10)
                    dec = true;
            }
        }

        if (color.equals("red")) {
            while (!dec) {
                if (colorSensor.red() > 10)
                    dec = true;
            }
        }

        if (color.equals("blue")) {
            while (!dec) {
                if (colorSensor.blue() > 10)
                    dec = true;
            }
        }
    }

    public void turnBySteps(double power, double inches) throws InterruptedException {

    	int[] startPosition = new int[4];

		for(DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		for(int i = 0; i < driveTrain.length; i++)
	    	startPosition[i] = driveTrain[i].getCurrentPosition();

		for(int i = 0; i < driveTrain.length; i++) {
            if (i % 2 == 1)
                driveTrain[i].setTargetPosition((int)(startPosition[i] + inches * COUNTS_PER_INCH)); // left motors
            else
                driveTrain[i].setTargetPosition((int)(startPosition[i] + -inches * COUNTS_PER_INCH)); // right motors
        }

	    for(DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		for(DcMotor motor : driveTrain)
    		motor.setPower(Math.abs(power));

		while(driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive()) {idle();}

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByAngle(double power, double angle) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        boolean turnComplete = false;

        navx_device.zeroYaw();
        yawPIDController.setSetpoint(angle);

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

                        for(DcMotor motor : driveTrain)
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
                /* A timeout occurred */
                    telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    turnBySteps(power, (neededInches + startPosition) - FrontLeft.getCurrentPosition());
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            }
        }
        catch(InterruptedException ex) {
             Thread.currentThread().interrupt();
        }
    }
}