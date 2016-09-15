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
@Autonomous(name="6322AutoTest", group="Autonomous")
//@Disabled


public class Test20166322 extends LinearOpModeCamera {

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;
    
    DcMotor[] driveTrain = {FrontRight, FrontLeft, BackRight, BackLeft};

    int ds2 = 2;  // additional downsampling of the image
				  // set to 1 to disable further downsampling

    //Switch for camera operation

    boolean limit = false;

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

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
    	FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
    	for (DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

    	for (DcMotor motor : driveTrain)
    		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition(),
                BackLeft.getCurrentPosition(),
                BackRight.getCurrentPosition());
        telemetry.update();

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();

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

        if (isCameraAvailable() && limit == true) {

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

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newLeftTarget);
            BackLeft.setTargetPosition(newLeftTarget);
            FrontRight.setTargetPosition(newRightTarget);
            BackRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FrontLeft.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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