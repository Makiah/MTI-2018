package org.firstinspires.ftc.teamcode.robot.hardware;

import hankutanku.math.angle.Angle;

public class IncrementalAbsoluteEncoder
{
    private final AbsoluteEncoder absoluteEncoder;
    private double totalAngularOffset = 0.0;
    private Angle previousEncoderAngle = null;

    public IncrementalAbsoluteEncoder(AbsoluteEncoder absoluteEncoder)
    {
        this.absoluteEncoder = absoluteEncoder;
    }

    public void updateIncremental()
    {
        if (previousEncoderAngle == null)
        {
            previousEncoderAngle = absoluteEncoder.heading();
            return;
        }

        totalAngularOffset += previousEncoderAngle.quickestDegreeMovementTo(absoluteEncoder.heading());
    }

    public double getTotalAngularOffset()
    {
        return totalAngularOffset;
    }
}
