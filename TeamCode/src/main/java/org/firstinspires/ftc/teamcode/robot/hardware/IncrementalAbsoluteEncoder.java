package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import dude.makiah.androidlib.logging.LoggingBase;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.Vector;

@Config
public class IncrementalAbsoluteEncoder
{
    private final AbsoluteEncoder absoluteEncoder;
    private double totalDegreeOffset = 0.0;
    private double currentAngularVelocity = 0.0;
    private long lastTimeCheck = 0;
    private Angle previousEncoderAngle = null;

    public IncrementalAbsoluteEncoder(AbsoluteEncoder absoluteEncoder)
    {
        this.absoluteEncoder = absoluteEncoder;
        LoggingBase.instance.lines("Got new");
    }

    public boolean angleBetween(Angle angle, Angle range1, Angle range2)
    {
        return Math.signum(angle.quickestDegreeMovementTo(range1)) != Math.signum(angle.quickestDegreeMovementTo(range2));
    }

    public void updateIncremental()
    {
        Angle currentHeading = absoluteEncoder.heading();
        long currentTime = System.currentTimeMillis();

        // Try to filter out noise
        if (previousEncoderAngle != null)
        {
            double deltaSeconds = (1.0 * currentTime - lastTimeCheck) / 1000.0;

            double projectedDegreeChangeFromLastLoop = currentAngularVelocity * deltaSeconds;
            Angle expectedHeading = previousEncoderAngle.add(new DegreeAngle(projectedDegreeChangeFromLastLoop));

            double bestRouteSuggestedByHeading = previousEncoderAngle.quickestDegreeMovementTo(currentHeading);
            double bestRouteSuggestedByVelocity = expectedHeading.quickestDegreeMovementTo(currentHeading);

            if (Math.abs(bestRouteSuggestedByHeading) > 90)
            {
                LoggingBase.instance.lines(Vector.decimalFormat.format(previousEncoderAngle.degrees()) + " => " + Vector.decimalFormat.format(currentHeading.degrees()) + " = " + Vector.decimalFormat.format(bestRouteSuggestedByHeading) + ", prediction was " + expectedHeading.degrees());
            }

            double deltaDegrees = 0.0;
            if (!angleBetween(currentHeading, previousEncoderAngle, expectedHeading) && Math.signum(bestRouteSuggestedByVelocity) != Math.signum(bestRouteSuggestedByHeading) && Math.abs(bestRouteSuggestedByVelocity) < Math.abs(bestRouteSuggestedByHeading)) // means that we're slowing down
            {
                deltaDegrees = 360 - bestRouteSuggestedByHeading;
                LoggingBase.instance.lines(Vector.decimalFormat.format(previousEncoderAngle.degrees()) + " => " + Vector.decimalFormat.format(currentHeading.degrees()) + " = " + Vector.decimalFormat.format(bestRouteSuggestedByHeading) + ", prediction was " + expectedHeading.degrees());
            }
            else {
                deltaDegrees = bestRouteSuggestedByHeading;
            }

            currentAngularVelocity = deltaDegrees / deltaSeconds;
            totalDegreeOffset += deltaDegrees;
        }

        lastTimeCheck = currentTime;
        previousEncoderAngle = currentHeading;
    }

    public void resetEncoderWheel()
    {
        previousEncoderAngle = null;
        currentAngularVelocity = 0;
        lastTimeCheck = 0;
        totalDegreeOffset = 0;
    }

    public double getTotalDegreeOffset()
    {
        return totalDegreeOffset;
    }

    public double getCurrentAngularVelocity()
    {
        return currentAngularVelocity;
    }
}
