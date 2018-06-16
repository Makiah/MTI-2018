package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import dude.makiah.androidlib.threading.ScheduledTask;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.hardware.HardwareInitializer;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;

/**
 * Note that we want to run the drive motors at full speed regardless of whether it's not perfectly
 * aligned with the mecanum wheels, so we'll have to modify the dot product.
 */
public class SpeedyMecanumDrive extends ScheduledTask
{
    private final Vector[] driveMotorOrientations = {
            new PolarVector(1, new DegreeAngle(45)),
            new PolarVector(1, new DegreeAngle(135)),
            new PolarVector(1, new DegreeAngle(225)),
            new PolarVector(1, new DegreeAngle(315))
    };
    private final DcMotor[] driveMotors; // clockwise orientation, starting with front right.

    public SpeedyMecanumDrive(DcMotor frontLeft, DcMotor backLeft, DcMotor backRight, DcMotor frontRight)
    {
        this.driveMotors = new DcMotor[]{frontLeft, backLeft, backRight, frontRight};
    }

    // The drive vector for the drivetrain (magnitude and angle).
    private Vector driveVector = null;
    public void setDriveVector(Vector driveVector)
    {
        if (driveVector.magnitude() > .01)
            this.driveVector = driveVector;
        else
            this.driveVector = null;
    }

    // The turn speed of the drivetrain.
    private double turnSpeed = 0;
    public void setTurnSpeed(double turnSpeed)
    {
        this.turnSpeed = turnSpeed;
    }

    /**
     * When drive vector is 45 degrees, we want to have
     *  front right: 0 power, back right: 1 power, back left: 0 power, front right: 1 power.
     *
     * When drive vector is zero degrees, we want to have
     *  front right: 1 power, back right: 1 power, back left: 1 power, front right: 1 power
     *
     * When drive vector is 22.5 degrees
     *
     * @return
     * @throws InterruptedException
     */
    @Override
    protected TimeMeasure onContinueTask() throws InterruptedException
    {
        double[] drivePowers = new double[4];

        // Dot product (cosine) to determine the power to apply to each wheel.
        for (int i = 0; i < driveMotors.length; i++)
            drivePowers[i] = driveVector.dot(driveMotorOrientations[i]);

        // Normalize the vectors to ensure we're moving at max speed allowed by the drive vector.
        double largestDrivePower = 0;
        for (double drivePower : drivePowers)
            if (Math.abs(drivePower) > largestDrivePower)
                largestDrivePower = drivePower;

        double powerIncreaseFactor = driveVector.magnitude() / largestDrivePower;
        for (int i = 0; i < drivePowers.length; i++)
            drivePowers[i] *= powerIncreaseFactor;

        for (int i = 0; i < drivePowers.length; i++)
            driveMotors[i].setPower(drivePowers[i]);

        // Update every 50 ms
        return new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 50);
    }
}
