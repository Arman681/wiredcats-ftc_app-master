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
@Autonomous(name="Gen3RedAuto", group="Autonomous")
//@Disabled

public class Gen3RedAuto extends LinearOpModeCamera {

    final DcMotor[] driveTrain = new DcMotor[4];

    //Drive Train Motor Declarations
    DcMotor frontleft, backleft;
    DcMotor frontright, backright;

    //Shooting Mechanism Motor Declarations
    DcMotor right, left;

    //Particle System Motor Declarations
    DcMotor intake, conveyor;

    //Servo Button Pusher Declaration
    Servo rightPusher;
    Servo leftPusher;

    @Override
    public void runOpMode() throws InterruptedException {

        //Drive Train Motors
        frontleft = hardwareMap.dcMotor.get("fl");
        backleft = hardwareMap.dcMotor.get("bl");
        frontright = hardwareMap.dcMotor.get("fr");
        backright = hardwareMap.dcMotor.get("br");
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);

        driveTrain[0] = frontright;
        driveTrain[1] = frontleft;
        driveTrain[2] = backright;
        driveTrain[3] = backleft;

        //Shooting Mechanism Motors
        right = hardwareMap.dcMotor.get("r");
        left = hardwareMap.dcMotor.get("l");
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);

        //Intake and Conveyor Motors
        intake = hardwareMap.dcMotor.get("i");
        conveyor = hardwareMap.dcMotor.get("c");
        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);

        //Button Pusher Servos
        rightPusher = hardwareMap.servo.get("rp");
        leftPusher = hardwareMap.servo.get("lp");

        waitForStart();
        moveByTime(0.0, 10000);
        moveByTime(0.2, 3500);

    }

    public void moveByTime(double power, int time) throws InterruptedException {

        for(DcMotor motor : driveTrain)
            motor.setPower(power);

        sleep(time);

        for(DcMotor motor : driveTrain)
            motor.setPower(0);
    }

    /*public void moveUntil(double power, String color) throws InterruptedException {

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
            while (!dec) {
                if (((CSleft.red() * 8) > (CSleft.blue() * 8)) || ((CSleft.red() * 8) > 4))
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
    }*/

    /*public void moveBySteps(double power, double inches) throws InterruptedException {

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

    }*/

    /*public void turnBySteps(double power, double inches) throws InterruptedException {

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
    }*/

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