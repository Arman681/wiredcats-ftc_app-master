package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    int ds2 = 2;  // additional downsampling of the image
				  // set to 1 to disable further downsampling

    //IMU setup

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TOLERANCE_DEGREES = 1.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    //encoder constants

    static final double TAU                  = 6.283185;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_RADIUS_INCHES  = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * TAU);

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

    public void timedMovement(double power, int time) throws InterruptedException {

		for(DcMotor motor : driveTrain)
    		motor.setPower(power);    	

    	sleep(time);

		for(DcMotor motor : driveTrain)
    		motor.setPower(0);
    }

    public void moveBySteps(double power, int inches) throws InterruptedException {

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
    }

    public void turnByAngle(double power, double angle) throws InterruptedException {

        boolean turnComplete = false;
        navx_device.zeroYaw();
        yawPIDController.setSetpoint(angle);

        try {
            yawPIDController.enable(true);

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;

            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            while ( (runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted() && !turnComplete) {
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
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            }
        }
        catch(InterruptedException ex) {
             Thread.currentThread().interrupt();
        }
    }
}