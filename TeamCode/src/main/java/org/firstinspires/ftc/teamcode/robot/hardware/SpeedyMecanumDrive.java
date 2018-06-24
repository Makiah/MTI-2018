package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousSettings;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TimeMeasure;
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

            // TODO test whether this works
//            drivePowers[0] *= 1.2;
//            drivePowers[1] *= 1.2;

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

    public void stop()
    {
        move(null, 0);
    }

    /**
     * Drives the robot by some vector and turn speed for some length of time
     * @param fieldCentricDrive  vector to drive
     * @param turnSpeed  turn speed
     * @param driveTime  time to drive
     * @throws InterruptedException
     */
    public void driveTime(PoseTrackingEncoderWheelSystem ptews, Vector fieldCentricDrive, double turnSpeed, TimeMeasure driveTime, Flow flow) throws InterruptedException
    {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < driveTime.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            move(fieldCentricDrive.rotateBy(ptews.getCurrentPose().heading.negative()), turnSpeed);
            ptews.update();
            flow.yield();
        }

        stop();
    }

    /**
     * Drives the robot by some vector and turn speed for some length of time
     * @param fieldCentricDrive  vector to drive
     * @param desiredHeading  desired heading during drive
     * @param driveTime  time to drive
     * @throws InterruptedException
     */
    public void driveTime(PoseTrackingEncoderWheelSystem ptews, Vector fieldCentricDrive, Angle desiredHeading, TimeMeasure driveTime, Flow flow) throws InterruptedException
    {
        long start = System.currentTimeMillis();

        boolean firstLine = true;

        while (System.currentTimeMillis() - start < driveTime.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            Vector fieldMovement = fieldCentricDrive;
            double turnSpeed = desiredHeading.quickestDegreeMovementTo(ptews.getCurrentPose().heading) / 20.0;

            if (firstLine)
                LoggingBase.instance.lines("Field Movement is " + fieldMovement.toString(false), "Turn speed is " + turnSpeed);

            move(fieldMovement, turnSpeed);
            ptews.update();
            flow.yield();

            firstLine = false;
        }

        stop();
    }

    public void turnToHeading(PoseTrackingEncoderWheelSystem ptews, Angle heading, double acceptableDifferenceDegrees, TimeMeasure timeout, Flow flow) throws InterruptedException
    {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeout.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            move(null, -1 * Range.clip(ptews.getCurrentPose().heading.quickestDegreeMovementTo(heading) / 20, -AutonomousSettings.maxTurnSpeed, AutonomousSettings.maxTurnSpeed));
            ptews.update();

            if (Math.abs(ptews.getCurrentPose().heading.quickestDegreeMovementTo(heading)) < acceptableDifferenceDegrees)
                break;

            flow.yield();
        }
        stop();
    }

    /**
     * Tells the mecanum drivetrain to match a given robot position and heading.
     * @param minimumInchesFromTarget  The maximum number of inches from the target pose which enables the robot to stop this code block.
     * @param minimumHeadingFromTarget  The maximum angle from the target pose which enables the robot to stop this code block.
     * @param timeoutLength  The maximum angle from the target pose which enables the robot to stop this code block.
     * @param completionBasedFunction  A function which returns void and receives a 0-1 input representing how close we are from execution completion.
     */
    public void matchPTEWSDesiredPose(PoseTrackingEncoderWheelSystem ptews, double minimumInchesFromTarget, Angle minimumHeadingFromTarget, TimeMeasure timeoutLength, Function<Void, Double> completionBasedFunction, Flow flow) throws InterruptedException
    {
        LoggingBase log = LoggingBase.instance;

        ProcessConsole console = log.newProcessConsole("Match Pose");

        // Relative means relative to the current robot pose.
        Vector previousPositionalOffset = null;
        double previousHeadingOffset = Double.NaN;

        double movementPowerUpFactor = 0;
        double headingPowerUpFactor = 0;

        double originalTargetOffset = Double.NaN;

        long start = System.currentTimeMillis();

        while (true)
        {
            if (System.currentTimeMillis() - start > timeoutLength.durationIn(TimeMeasure.Units.MILLISECONDS))
            {
                LoggingBase.instance.lines("Timed out");
                break;
            }

            ptews.update();

            // Current position and heading from tracking encoders
            Pose currentPose = ptews.getCurrentPose();
            if (Double.isNaN(originalTargetOffset))
                originalTargetOffset = currentPose.position.magnitude();

            // Offset from target position
            Vector positionalOffsetFromTarget = ptews.getDesiredPose().position.subtract(currentPose.position);
            double headingDegreeOffsetFromTarget = ptews.getDesiredPose().heading.quickestDegreeMovementTo(currentPose.heading);

            // Whether we can cancel the movement.
            if (positionalOffsetFromTarget.magnitude() <= minimumInchesFromTarget && Math.abs(headingDegreeOffsetFromTarget) <= minimumHeadingFromTarget.degrees())
            {
                LoggingBase.instance.lines("Quit movement because position offset is " + positionalOffsetFromTarget.toString(false) + " and heading offset is " + headingDegreeOffsetFromTarget);
                break;
            }

            if (positionalOffsetFromTarget.magnitude() <= minimumInchesFromTarget)
            {
                positionalOffsetFromTarget = new CartesianVector(0, 0);
            }

            if (Math.abs(headingDegreeOffsetFromTarget) <= minimumHeadingFromTarget.degrees())
            {
                headingDegreeOffsetFromTarget = 0;
            }

            // Calculate how to move toward the target pose.
            Vector driveVector = positionalOffsetFromTarget.rotateBy(currentPose.heading.negative()).divide(20);
            driveVector = new PolarVector(Range.clip(driveVector.magnitude(), -AutonomousSettings.maxMoveSpeed, AutonomousSettings.maxMoveSpeed), driveVector.angle());
            double turnSpeed = Range.clip(headingDegreeOffsetFromTarget / 50, -AutonomousSettings.maxTurnSpeed, AutonomousSettings.maxTurnSpeed);
            move(driveVector, turnSpeed);

            // Determine whether we should power movement up or down.
            if (previousPositionalOffset != null)
            {
                Vector movedSinceLastLoop = positionalOffsetFromTarget.subtract(previousPositionalOffset);
                movementPowerUpFactor += (.03 - (movedSinceLastLoop.magnitude() / previousPositionalOffset.magnitude())) * .5; // should have moved 10 percent more of the way toward our target destination.
            }
            previousPositionalOffset = positionalOffsetFromTarget;

            // Determine whether we should power turning up or down.
            if (!Double.isNaN(previousHeadingOffset))
            {
                double degreesMovedSinceLastLoop = new DegreeAngle(headingDegreeOffsetFromTarget).quickestDegreeMovementTo(new DegreeAngle(previousHeadingOffset));
                headingPowerUpFactor += (.03 - (Math.abs(degreesMovedSinceLastLoop) / Math.abs(previousHeadingOffset))) * .06; // should have moved 10 percent more of the required heading.
            }
            previousHeadingOffset = headingDegreeOffsetFromTarget;

            // Call any user-defined lambdas.
            if (completionBasedFunction != null)
                completionBasedFunction.value((1.0 - positionalOffsetFromTarget.magnitude()) / originalTargetOffset);

            // Write console data
            console.write(
                    "Target pose is " + ptews.getCurrentPose().toString(),
                    "Positional offset is " + positionalOffsetFromTarget.toString(false),
                    "Heading offset is " + Vector.decimalFormat.format(headingDegreeOffsetFromTarget),
                    "Movement power up is " + movementPowerUpFactor,
                    "Heading power up is " + headingPowerUpFactor);

            // Pause so that we don't update instantaneously.
            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 30));
        }

        stop();

        console.destroy();
    }
}
