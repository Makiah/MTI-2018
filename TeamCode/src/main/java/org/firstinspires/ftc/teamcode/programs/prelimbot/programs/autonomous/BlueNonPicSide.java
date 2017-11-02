package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.logging.Log;
import hankextensions.threading.Flow;

/**
 * Created by JordanArnold on 10/3/17.
 *
 *
 * For Autonomous
 *
 *
 * Use gyro for initial position
 * Hit Jewel
 * Read image
 * deposit
 *
 *
 */

@Autonomous(name="BlueNonPicSide")
public class BlueNonPicSide extends BaseAuto
{
    @Override
    protected void START() throws InterruptedException
    {
        while(true) {

            console.write("Red");
            console.write(Integer.toString(colorSensor.red()));
            console.write("Blue");
            console.write(Integer.toString(colorSensor.blue()));
            console.write("Green");
            console.write(Integer.toString(colorSensor.green()));


            Flow.yield();
        }
    }
}
