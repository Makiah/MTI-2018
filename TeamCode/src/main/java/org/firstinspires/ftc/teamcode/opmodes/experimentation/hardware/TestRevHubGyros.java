package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
                hardwareMap.get(BNO055IMU.class, "imu"),
                hardwareMap.get(BNO055IMU.class, "imu 1"),
                flow
        );

        ProcessConsole console = log.newProcessConsole("Hub Gyros");
        while (true)
        {
            Angle[][] readingsRaw = gyros.hubAngles();
            Angle[] readingsAveraged = gyros.hubAnglesAveraged();

            console.write("Rev Hub Gyros: ",
                    "Hub 1 | x: " + Vector.decimalFormat.format(readingsRaw[0][0].degrees()) + ", y: " + Vector.decimalFormat.format(readingsRaw[0][1].degrees()) + ", z: " + Vector.decimalFormat.format(readingsRaw[0][2].degrees()),
                    "Hub 2 | x: " + Vector.decimalFormat.format(readingsRaw[1][0].degrees()) + ", y: " + Vector.decimalFormat.format(readingsRaw[1][1].degrees()) + ", z: " + Vector.decimalFormat.format(readingsRaw[1][2].degrees()),
                    "Averaged | x: " + Vector.decimalFormat.format(readingsAveraged[0].degrees()) + ", y: " + Vector.decimalFormat.format(readingsAveraged[1].degrees()) + ", z: " + Vector.decimalFormat.format(readingsAveraged[2].degrees()));

//            Orientation gyro1 = gyros.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            Orientation gyro2 = gyros.gyro2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//            console.write("Rev Hub Gyros: ",
//                    "Hub 1 | x: " + gyro1.firstAngle + ", y: " + gyro1.secondAngle + ", z: " + gyro1.thirdAngle,
//                    "Hub 2 | x: " + gyro2.firstAngle + ", y: " + gyro2.secondAngle + ", z: " + gyro2.thirdAngle);

            flow.yield();
        }
    }
}
