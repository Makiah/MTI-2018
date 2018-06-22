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

@Autonomous(name="Test Rev Sensor", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestColorSensor extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        ColorSensor sensorColor = hardware.initialize(ColorSensor.class, "glyph sensor");;
        DistanceSensor sensorDistance = hardware.initialize(DistanceSensor.class, "glyph sensor");
        ColorSensor sensorColor1 = hardware.initialize(ColorSensor.class, "glyph sensor 2");
        DistanceSensor sensorDistance1 = hardware.initialize(DistanceSensor.class, "glyph sensor 2");

        ProcessConsole console = log.newProcessConsole("Jewel Sensor Data");

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (true)
        {
            console.write(
                    "Distance (cm): " + String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)),
                    "Alpha: " + sensorColor.alpha(),
                    "Red: " + sensorColor.red(),
                    "Green: " + sensorColor.green(),
                    "Blue: " + sensorColor.blue(),
                    "====",
                    "Distance (cm): " + String.format(Locale.US, "%.02f", sensorDistance1.getDistance(DistanceUnit.CM)),
                    "Alpha: " + sensorColor1.alpha(),
                    "Red: " + sensorColor1.red(),
                    "Green: " + sensorColor1.green(),
                    "Blue: " + sensorColor1.blue());

            flow.yield();
        }
    }
}
