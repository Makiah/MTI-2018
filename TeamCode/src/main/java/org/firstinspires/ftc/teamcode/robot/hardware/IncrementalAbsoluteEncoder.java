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

            double deltaDegrees = 0.0;

            double bestRouteSuggestedByVelocity = expectedHeading.quickestDegreeMovementTo(currentHeading);
            double bestRouteSuggestedByHeading = previousEncoderAngle.quickestDegreeMovementTo(currentHeading);

            if (!angleBetween(currentHeading, previousEncoderAngle, expectedHeading) && Math.signum(bestRouteSuggestedByVelocity) != Math.signum(bestRouteSuggestedByHeading) && Math.abs(bestRouteSuggestedByVelocity) < Math.abs(bestRouteSuggestedByHeading)) // means that we're slowing down
            {
                System.out.println("Better off following the route suggested by current velocity");
                deltaDegrees = 360 - bestRouteSuggestedByHeading;
            }
            else
            {
                System.out.println("Better off following the route suggested by current heading");
                deltaDegrees = bestRouteSuggestedByHeading;
            }

            double previousAngularVelocity = currentAngularVelocity;

            currentAngularVelocity = deltaDegrees / deltaSeconds;

            if (Math.abs(currentAngularVelocity) > 90)
            {
                LoggingBase.instance.lines("FUCK. Ok, so dt = " + Vector.decimalFormat.format(deltaSeconds) + " velocity was " + Vector.decimalFormat.format(previousAngularVelocity) + " so expected heading was " + Vector.decimalFormat.format(expectedHeading.degrees()) + " p encoder " + Vector.decimalFormat.format(previousEncoderAngle.degrees()) + " now encoder " + Vector.decimalFormat.format(currentHeading.degrees()) + " da = " + Vector.decimalFormat.format(deltaDegrees) + " which made velocity " + Vector.decimalFormat.format(currentAngularVelocity));
            }

            totalDegreeOffset += deltaDegrees;
        }

        lastTimeCheck = System.currentTimeMillis();
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
