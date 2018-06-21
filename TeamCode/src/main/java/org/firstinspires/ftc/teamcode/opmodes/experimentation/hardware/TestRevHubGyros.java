package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.DoubleRevGyro;

import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.vector.Vector;

@Autonomous(name="Test Rev Hub Gyros", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestRevHubGyros extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        DoubleRevGyro gyros = new DoubleRevGyro(
                hardwareMap.get(BNO055IMU.class, "imu1"),
                hardwareMap.get(BNO055IMU.class, "imu2"),
                flow
        );

        ProcessConsole console = log.newProcessConsole("Hub Gyros");
        while (true)
        {
            Angle[][] readingsRaw = gyros.hubAngles();
            Angle[] readingsAveraged = gyros.hubAnglesAveraged();

            console.write("Rev Hub Gyros: ",
                    "Hub 1 | x: " + Vector.decimalFormat.format(readingsRaw[0][0]) + ", y: " + Vector.decimalFormat.format(readingsRaw[0][1]) + ", z: " + Vector.decimalFormat.format(readingsRaw[0][2]),
                    "Hub 2 | x: " + Vector.decimalFormat.format(readingsRaw[1][0]) + ", y: " + Vector.decimalFormat.format(readingsRaw[1][1]) + ", z: " + Vector.decimalFormat.format(readingsRaw[1][2]),
                    "Averaged | x: " + Vector.decimalFormat.format(readingsAveraged[0]) + ", y: " + Vector.decimalFormat.format(readingsAveraged[1]) + ", z: " + Vector.decimalFormat.format(readingsAveraged[2]));

            flow.yield();
        }
    }
}
