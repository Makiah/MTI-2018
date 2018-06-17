package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;

/**
 * Note that we want to run the drive motors at full speed regardless of whether it's not perfectly
 * aligned with the mecanum wheels, so we'll have to modify the dot product.
 */
public class SpeedyMecanumDrive
{
    private static final Vector[] driveMotorOrientations = {
            new PolarVector(1, new DegreeAngle(45)),
            new PolarVector(1, new DegreeAngle(135)),
            new PolarVector(1, new DegreeAngle(225)),
            new PolarVector(1, new DegreeAngle(315))
    };
    private final DcMotor[] driveMotors; // clockwise orientation, starting with front right.

    public SpeedyMecanumDrive(DcMotor frontLeft, DcMotor backLeft, DcMotor backRight, DcMotor frontRight)
    {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.driveMotors = new DcMotor[]{frontLeft, backLeft, backRight, frontRight};
    }

    public void move(Vector driveVector, double turnSpeed)
    {
        double[] drivePowers = new double[4];

        // Dot product (cosine) to determine the power to apply to each wheel.
        for (int i = 0; i < driveMotors.length; i++)
            drivePowers[i] = (driveVector != null ? driveVector.dot(driveMotorOrientations[i]) : 0) + turnSpeed;

        if (driveVector != null)
        {
            // Normalize the vectors to ensure we're moving at max speed allowed by the drive vector.
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
