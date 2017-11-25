package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.RobotCore;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.phonesensors.Gyro;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;

@Autonomous(name = "Test Android Gyro", group = "Experimentation")
public class EnsureAndroidGyro extends RobotCore
{
    Gyro phoneGyro;

    @Override
    protected void INITIALIZE() throws InterruptedException {
        phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();
    }

    protected void START() throws InterruptedException
    {
        ProcessConsole gyroConsole = log.newProcessConsole("Phone Gyro");

        while (true)
        {
            gyroConsole.write(
                    "X: " + phoneGyro.x(),
                    "Y: " + phoneGyro.y(),
                    "Z: " + phoneGyro.z()
            );

            flow.yield();
        }
    }
}
