package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;

@Config
public class IncrementalAbsoluteEncoder
{
    public static double maxAngularVelocityInDegsPerSec = 270;

    private final AbsoluteEncoder absoluteEncoder;
    private double totalDegreeOffset = 0.0;
    private double currentAngularVelocity = 0.0;
    private long lastTimeCheck = 0;
    private Angle previousEncoderAngle = null;

    public IncrementalAbsoluteEncoder(AbsoluteEncoder absoluteEncoder)
    {
        this.absoluteEncoder = absoluteEncoder;
    }

    public void updateIncremental()
    {
        if (previousEncoderAngle != null)
        {
            double deltaSeconds = (1.0 * System.currentTimeMillis() - lastTimeCheck) / 1000.0;
            double projectedDegreeChangeFromLastLoop = currentAngularVelocity * deltaSeconds;
            Angle expectedHeading = previousEncoderAngle.add(new DegreeAngle(projectedDegreeChangeFromLastLoop));
            Angle currentHeading = absoluteEncoder.heading();

            double deltaDegrees = currentHeading.subtract(previousEncoderAngle).degrees();

            if (expectedHeading.quickestDegreeMovementTo(currentHeading) > 0) // means that we're slowing down
                deltaDegrees = 360 - deltaDegrees;
            else
                deltaDegrees = -deltaDegrees;

            if (deltaDegrees > 180)
                deltaDegrees -= 360;

            currentAngularVelocity = Range.clip(deltaDegrees / deltaSeconds, -maxAngularVelocityInDegsPerSec, maxAngularVelocityInDegsPerSec);
            totalDegreeOffset += deltaDegrees;
        }

        lastTimeCheck = System.currentTimeMillis();
        previousEncoderAngle = absoluteEncoder.heading();
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
