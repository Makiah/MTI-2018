package org.firstinspires.ftc.teamcode.robot.hardware;

import hankutanku.math.angle.Angle;

public class IncrementalAbsoluteEncoder
{
    private final AbsoluteEncoder absoluteEncoder;
    private double totalDegreeOffset = 0.0;
    private Angle previousEncoderAngle = null;

    public IncrementalAbsoluteEncoder(AbsoluteEncoder absoluteEncoder)
    {
        this.absoluteEncoder = absoluteEncoder;
    }

    public void updateIncremental()
    {
        if (previousEncoderAngle != null)
        {
            // TODO make more complex, and thus more accurate
            totalDegreeOffset += previousEncoderAngle.quickestDegreeMovementTo(absoluteEncoder.heading());
        }

        previousEncoderAngle = absoluteEncoder.heading();
    }

    public double getTotalDegreeOffset()
    {
        return totalDegreeOffset;
    }
}
