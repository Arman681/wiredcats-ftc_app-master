package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
@Autonomous(name="6322AutoTest", group="Autonomous")
//@Disabled

public class Test20166322 extends LinearOpModeCamera {

    //Drive Train Motor Declarations
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    final DcMotor[] driveTrain = new DcMotor[4];

    //Color Sensor Declarations
    ColorSensor CSleft;
    ColorSensor CSright;

    //Optical Distance Sensor Declarations
    OpticalDistanceSensor ODSleft;
    OpticalDistanceSensor ODSright;

    //Continuous Rotation Servo Declarations
    CRServo rightPusher;
    CRServo leftPusher;

    int bnum = 0;
    int ds2 = 2;  // additional downsampling of the image

    float hsvValues[] = {0F,0F,0F};

    //IMU setup
    //AHRS navx_device;
    //navXPIDController yawPIDController;

    final int NAVX_DIM_I2C_PORT = 0;

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

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);

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

        //Continuous Rotation Sensors
        rightPusher = hardwareMap.crservo.get("rp");
        leftPusher = hardwareMap.crservo.get("lp");

        /*navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                      NAVX_DIM_I2C_PORT,
                      AHRS.DeviceDataType.kProcessedData,
                      NAVX_DEVICE_UPDATE_RATE_HZ);

        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);*/

        waitForStart();
        moveBySteps(0.8, 22);
        turnBySteps(0.6, -22);
        moveBySteps(0.8, 44);
        turnBySteps(0.6, 22);
        moveUntil(0.7, "red");

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

        while (opModeIsActive()) {

            telemetry.addData("Iterations " + ++c, null);
            telemetry.addData("LED", true ? "On" : "Off");
            telemetry.addData("Red  ", CSleft.red()*8);
            telemetry.addData("Blue ", CSleft.blue()*8);
            telemetry.update();

            for (DcMotor motor : driveTrain)
                motor.setPower(power);

            if (color.equals("white"))
                while (!dec)
                    if (CSleft.red() > 8 && CSleft.green() > 8 && CSleft.blue() > 8)
                        dec = true;

            if (color.equals("red"))
                while (!dec)
                    if ((CSleft.red()*8) > (CSleft.blue()*8))
                        dec = true;

            if (color.equals("blue"))
                while (!dec)
                    if ((CSleft.blue()*8) > (CSleft.red()*8))
                        dec = true;

            for (DcMotor motor : driveTrain)
                motor.setPower(0);
            idle();
            sleep(100);
        }
    }

    public void moveBySteps(double power, double inches) throws InterruptedException {

        int[] startPosition = new int[4];

        for (DcMotor motor : driveTrain)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    //Uses gyroscopic features in the NAVX Micro Sensor
    /*public void turnByAngle(double power, double angle) throws InterruptedException {

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

                    } else {

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
    }*/

    //Determines the color of the button using the ZTE Camera
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