package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    DcMotor[] motor = {FrontRight, FrontLeft, BackRight, BackLeft};

    int ds2 = 2;  // additional downsampling of the image
				  // set to 1 to disable further downsampling

    @Override
    public void runOpMode() throws InterruptedException {

    	FrontRight = hardwareMap.dcMotor.get("FrontRight");
    	FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
    	BackRight = hardwareMap.dcMotor.get("BacktRight");
    	BackLeft = hardwareMap.dcMotor.get("BacktLeft");

    	FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    	BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    	FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    	BackLeft.setDirection(DcMotor.Direction.REVERSE);

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

    	FrontRight.setPower(power);
    	FrontLeft.setPower(power);
    	BackRight.setPower(power);
    	BackLeft.setPower(power);

    	sleep(time);

    	FrontRight.setPowerFloat();
    	FrontLeft.setPowerFloat();
    	BackRight.setPowerFloat();
    	BackLeft.setPowerFloat();
    }

    public void MoveBySteps(double power, int inches) throws InterruptedException {

    	int startPosition;

    	for(int i = 0; i < motor.length; i++)
    	{
    		motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    	}

    	startPosition = FrontRight.getCurrentPosition();

    	FrontRight.setTargetPosition(inches + startPosition);

    	FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    	FrontRight.setPower(power);
    	FrontLeft.setPower(power);
    	BackRight.setPower(power);
    	BackLeft.setPower(power);

    	while(FrontRight.isBusy()) {}
    }
}