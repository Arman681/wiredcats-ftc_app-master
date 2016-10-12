package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
@Autonomous(name="TestSensors", group="Autonomous")
//@Disabled

public class TestSensors extends LinearOpModeCamera {

    //Color Sensor Declarations
    ColorSensor CS;

    //View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;



    @Override
    public void runOpMode() throws InterruptedException {

        //Color Sensors
        CS = hardwareMap.colorSensor.get("cs");
        CS.enableLed(false);
        CS.enableLed(true);


        waitForStart();
        int c = 0;
        while (opModeIsActive()) {
            Color.RGBToHSV(CS.red() * 8, CS.green() * 8, CS.blue() * 8, hsvValues);
            telemetry.addData("Iterations " + ++c, null);
            telemetry.addData("LED", true ? "On" : "Off");
            telemetry.addData("Red ", (CS.red()*8));
            telemetry.addData("Green", (CS.green()*8));
            telemetry.addData("Blue ", (CS.blue()*8));
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();

            idle();

        }

    }

}