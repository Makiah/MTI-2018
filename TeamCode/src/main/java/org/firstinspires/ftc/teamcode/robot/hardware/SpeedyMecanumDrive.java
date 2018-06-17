package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;

/**
 * Note that we want to run the drive motors at full speed regardless of whether it's not perfectly
 * aligned with the mecanum wheels, so we'll have to modify the dot product.
 */
public class SpeedyMecanumDrive
{
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(18, 18)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private final DcMotor[] driveMotors; // frontLeft, backLeft, backRight, frontRight respectively.

    public SpeedyMecanumDrive(DcMotor frontLeft, DcMotor backLeft, DcMotor backRight, DcMotor frontRight)
    {
        this.driveMotors = new DcMotor[]{frontLeft, backLeft, backRight, frontRight};
    }

    public void move(Vector driveVector, double turnSpeed)
    {
        double[] drivePowers = new double[4];

        for (int i = 0; i < drivePowers.length; i++)
            drivePowers[i] = turnSpeed;

        if (driveVector != null)
        {
            // Dot product (cosine) to determine the power to apply to each wheel.
            for (int i = 0; i < driveMotors.length; i++)
                drivePowers[i] = Math.cos(driveVector.angle().degrees() - WHEEL_ORIENTATIONS[i]);

            // Normalize the vectors to ensure we're moving at max speed allowed by the drive vector.
            // For example, traveling forward with classical code results in .7 on each motor, this ensures that we actually move at 1 power (max speed).
            double largestDrivePower = 0;
            for (double drivePower : drivePowers)
                if (Math.abs(drivePower) > largestDrivePower)
                    largestDrivePower = drivePower;

            double powerIncreaseFactor = driveVector.magnitude() / largestDrivePower;
            for (int i = 0; i < drivePowers.length; i++)
                drivePowers[i] *= powerIncreaseFactor;
        }

        for (int i = 0; i < drivePowers.length; i++)
            driveMotors[i].setPower(drivePowers[i]);
    }
}
