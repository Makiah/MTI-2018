package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import java.util.Locale;

import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.EnhancedOpMode;

@Autonomous(name="Test Color Sensor", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestColorSensor extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        ColorSensor sensorColor = hardware.initialize(ColorSensor.class, "jewel sensor");;
        DistanceSensor sensorDistance = hardware.initialize(DistanceSensor.class, "jewel sensor");;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        ProcessConsole console = log.newProcessConsole("Jewel Sensor Data");

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (true)
        {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            console.write(
                    "Distance (cm): " + String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)),
                    "Alpha: " + sensorColor.alpha(),
                    "Red: " + sensorColor.red(),
                    "Green: " + sensorColor.green(),
                    "Blue: " + sensorColor.blue(),
                    "Hue: " + hsvValues[0],
                    "Currently seeing: " + (hsvValues[0] > 310 ? "red" : "blue"));

            flow.yield();
        }
    }
}
