package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.threading.Flow;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;

public class DoubleRevGyro
{
    public final BNO055IMU gyro1, gyro2;

    public DoubleRevGyro(BNO055IMU gyro1, BNO055IMU gyro2, Flow flow) throws InterruptedException
    {
        BNO055IMU.Parameters p1 = new BNO055IMU.Parameters();
        p1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        p1.loggingEnabled = true;
        p1.loggingTag = "IMU1";
        p1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro1.initialize(p1);
        this.gyro1 = gyro1;

        LoggingBase.instance.lines("Calibrating gyro 1...");
        while (this.gyro1.isGyroCalibrated())
            flow.yield();
        LoggingBase.instance.lines("Complete!");

        BNO055IMU.Parameters p2 = new BNO055IMU.Parameters();
        p2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        p2.loggingEnabled = true;
        p2.loggingTag = "IMU2";
        p2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro2.initialize(p2);
        this.gyro2 = gyro2;

        LoggingBase.instance.lines("Calibrating gyro 2...");
        while (this.gyro2.isGyroCalibrated())
            flow.yield();
        LoggingBase.instance.lines("Complete!");
    }

    public Angle[][] hubAngles()
    {
        Orientation angles1 = gyro1.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        Orientation angles2 = gyro2.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return new Angle[][]{
                new Angle[] {new DegreeAngle(angles1.firstAngle), new DegreeAngle(angles1.secondAngle), new DegreeAngle(angles1.thirdAngle)},
                new Angle[] {new DegreeAngle(angles2.firstAngle), new DegreeAngle(angles2.secondAngle), new DegreeAngle(angles2.thirdAngle)}};
    }

    public Angle[] hubAnglesAveraged()
    {
        Angle[][] hubAngles = hubAngles();

        return new Angle[]{Angle.average(hubAngles[0][0], hubAngles[1][0]), Angle.average(hubAngles[0][1], hubAngles[1][1]), Angle.average(hubAngles[0][2], hubAngles[1][2])};
    }
}
