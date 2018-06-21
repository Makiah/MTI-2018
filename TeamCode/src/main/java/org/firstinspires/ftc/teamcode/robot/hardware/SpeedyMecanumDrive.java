package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.Flow;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.function.Function;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;

/**
 * Note that we want to run the drive motors at full speed regardless of whether it's not perfectly
 * aligned with the mecanum wheels, so we'll have to modify the dot product.
 */
public class SpeedyMecanumDrive
{
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(18, 18)); // Will be 45 degrees with perfect square dimensions.
    private static final Angle[] WHEEL_ORIENTATIONS = {
            new DegreeAngle(ROBOT_PHI - 90),
            new DegreeAngle((180 - ROBOT_PHI) - 90),
            new DegreeAngle((180 + ROBOT_PHI) - 90),
            new DegreeAngle((360 - ROBOT_PHI) - 90)
    };
    private final DcMotor[] driveMotors; // frontLeft, backLeft, backRight, frontRight respectively.

    private final ProcessConsole drivePowerConsole;

    public SpeedyMecanumDrive(DcMotor frontLeft, DcMotor backLeft, DcMotor backRight, DcMotor frontRight)
    {
        this.driveMotors = new DcMotor[]{frontLeft, backLeft, backRight, frontRight};

        for (DcMotor driveMotor : driveMotors)
            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // avoid braking motors for durability.

        this.drivePowerConsole = LoggingBase.instance.newProcessConsole("Mecanum Drive");
    }

    /**
     * With typical mecanum control, running the drive directly forward only powers each wheel
     * at sqrt(2) / 2 power forward (.707 approximately) because of how cos and sin are evaluated.
     * This normalizes them to actually drive at the full power allowed by the magnitude of the
     * drive vector.
     * @param driveVector The speed and heading at which to drive.
     * @param turnSpeed   The speed at which to turn the robot.
     */
    public void move(Vector driveVector, double turnSpeed)
    {
        if (driveVector == null || driveVector.magnitude() < .01)
            driveVector = null;
        else
            driveVector = driveVector.rotateBy(new DegreeAngle(-90));

        double[] drivePowers = new double[4];

        for (int i = 0; i < drivePowers.length; i++)
            drivePowers[i] = turnSpeed;

        if (driveVector != null)
        {
            // Dot product (cosine) to determine the power to apply to each wheel.
            for (int i = 0; i < driveMotors.length; i++)
                drivePowers[i] += Math.cos(driveVector.angle().subtract(WHEEL_ORIENTATIONS[i]).radians());

            // Normalize the vectors to ensure we're moving at max speed allowed by the drive vector.
            // For example, traveling forward with classical code results in .7 on each motor, this ensures that we actually move at 1 power (max speed).
            double largestDrivePower = 0;
            for (double drivePower : drivePowers)
                if (Math.abs(drivePower) > largestDrivePower)
                    largestDrivePower = Math.abs(drivePower);

            double powerIncreaseFactor = Range.clip(driveVector.magnitude() + Math.abs(turnSpeed), 0, 1) / largestDrivePower;
            for (int i = 0; i < drivePowers.length; i++)
                drivePowers[i] *= powerIncreaseFactor;
        }

        for (int i = 0; i < drivePowers.length; i++)
            driveMotors[i].setPower(drivePowers[i]);

        this.drivePowerConsole.write(
                "Front Left: " + drivePowers[0],
                "Back Left: " + drivePowers[1],
                "Back Right: " + drivePowers[2],
                "Front Right: " + drivePowers[3],
                "Drive Vector: : " + (driveVector != null ? driveVector.toString() : "NA"),
                "Turn: " + turnSpeed
        );
    }

    /**
     * Tells the mecanum drivetrain to match a given robot position and heading.
     * @param ptews  The tracking code which keeps track of the trig and changes in encoder heading.
     * @param desiredPose  The pose which the mecanum drive should attempt to reach.
     * @param minimumInchesFromTarget  The maximum number of inches from the target pose which enables the robot to stop this code block.
     * @param minimumHeadingFromTarget  The maximum angle from the target pose which enables the robot to stop this code block.
     * @param completionBasedFunction  A function which returns void and receives a 0-1 input representing how close we are from execution completion.
     */
    public void matchPose(String movementName,
                          PoseTrackingEncoderWheelSystem ptews,
                          Pose desiredPose,
                          double minimumInchesFromTarget,
                          Angle minimumHeadingFromTarget,
                          Function<Void, Double> completionBasedFunction,
                          Flow flow) throws InterruptedException
    {
        ProcessConsole console = LoggingBase.instance.newProcessConsole("Matching Pose");

        // Relative means relative to the current robot pose.
        if (desiredPose.poseType == Pose.PoseType.RELATIVE)
        {
            desiredPose = new Pose(Pose.PoseType.ABSOLUTE, ptews.getCurrentPose().position.add(desiredPose.position), ptews.getCurrentPose().heading.add(desiredPose.heading));
        }

        double originalTargetOffset = Double.NaN;

        while (true)
        {
            ptews.update();

            Pose currentPose = ptews.getCurrentPose();

            Vector positionalOffsetFromTarget = desiredPose.position.subtract(currentPose.position);

            if (Double.isNaN(originalTargetOffset))
                originalTargetOffset = positionalOffsetFromTarget.magnitude();

            double headingDegreeOffsetFromTarget = desiredPose.heading.quickestDegreeMovementTo(currentPose.heading);

            if (positionalOffsetFromTarget.magnitude() <= minimumInchesFromTarget && Math.abs(headingDegreeOffsetFromTarget) <= minimumHeadingFromTarget.degrees())
                break;

            move(positionalOffsetFromTarget.rotateBy(currentPose.heading.negative()).divide(50), headingDegreeOffsetFromTarget / 70);

            if (completionBasedFunction != null)
                completionBasedFunction.value((1.0 - positionalOffsetFromTarget.magnitude()) / originalTargetOffset);

            console.write(
                    movementName,
                    "Target pose is " + desiredPose.toString(),
                    "Positional offset is " + positionalOffsetFromTarget.toString(false),
                    "Heading offset is " + Vector.decimalFormat.format(headingDegreeOffsetFromTarget));

            flow.yield();
        }

        move(null, 0);

        flow.yield();
    }
}
