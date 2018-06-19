package org.firstinspires.ftc.teamcode.robot.hardware;

import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

public class PoseTrackingEncoderWheelSystem
{
    private final Angle headingChangeRequiredForPoseUpdate = new DegreeAngle(2); // If we keep track of every shift in noise then the data starts to drift.

    private final AbsoluteEncoder leftWheel, centerWheel, rightWheel; // These COULD be incremental, but we'd just be recalculating stuff.
    private Pose currentPose = new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(0, 0), new DegreeAngle(0)); // Reference changes over time.

    private Angle leftWheelCurrent = new DegreeAngle(0), centerWheelCurrent = new DegreeAngle(0), rightWheelCurrent = new DegreeAngle(0);
    private Angle leftWheelPrevious = new DegreeAngle(0), centerWheelPrevious = new DegreeAngle(0), rightWheelPrevious = new DegreeAngle(0);
    private double leftWheelDelta = 0.0, centerWheelDelta = 0.0, rightWheelDelta = 0.0;
    private double leftWheelCumulative = 0.0, rightWheelCumulative = 0.0;

    private final double robotSpinCircumference = 18 * Math.PI;
    private final double angularToPositionalConversionConstant = 1 / 360.0 * (Math.PI * 4);

    public PoseTrackingEncoderWheelSystem(AbsoluteEncoder leftWheel, AbsoluteEncoder centerWheel, AbsoluteEncoder rightWheel)
    {
        this.leftWheel = leftWheel;
        this.centerWheel = centerWheel;
        this.rightWheel = rightWheel;

        leftWheelPrevious = leftWheel.heading();
        centerWheelPrevious = centerWheel.heading();
        rightWheelPrevious = rightWheel.heading();
    }

    public void provideExternalPoseInformation(Pose info)
    {
        if (info.poseType == Pose.PoseType.ABSOLUTE)
            this.currentPose = info;
        else
        {
            this.currentPose.add(info);
        }
    }

    public void update()
    {
        leftWheelCurrent = leftWheel.heading();
        centerWheelCurrent = centerWheel.heading();
        rightWheelCurrent = rightWheel.heading();

        // Ensure that each delta reaches the minimum, otherwise ignore it.
        leftWheelDelta = leftWheelPrevious.quickestDegreeMovementTo(leftWheelCurrent);
        if (Math.abs(leftWheelDelta) < headingChangeRequiredForPoseUpdate.degrees())
            leftWheelDelta = 0;
        else
        {
            leftWheelDelta *= angularToPositionalConversionConstant;
            leftWheelPrevious = leftWheelCurrent;
        }

        centerWheelDelta = centerWheelPrevious.quickestDegreeMovementTo(centerWheelCurrent);
        if (Math.abs(centerWheelDelta) < headingChangeRequiredForPoseUpdate.degrees())
            centerWheelDelta = 0;
        else
        {
            leftWheelDelta *= angularToPositionalConversionConstant;
            centerWheelPrevious = centerWheelCurrent;
        }

        rightWheelDelta = rightWheelPrevious.quickestDegreeMovementTo(rightWheelCurrent);
        if (Math.abs(rightWheelDelta) < headingChangeRequiredForPoseUpdate.degrees())
            rightWheelDelta = 0;
        else
        {
            leftWheelDelta *= angularToPositionalConversionConstant;
            rightWheelPrevious = rightWheelCurrent;
        }

        // The extent to which the left and right wheels have different measurements, represents angular orientation difference.
        // Doesn't at all depend on previous measurements.
        leftWheelCumulative += leftWheelDelta;
        rightWheelCumulative += rightWheelDelta;
        Angle currentHeading = new DegreeAngle((rightWheelCumulative - leftWheelCumulative) / robotSpinCircumference * 180.0);

        // Now for position, which does in fact depend on previous movements.
        Vector positionDelta = new CartesianVector(centerWheelDelta, (rightWheelDelta + leftWheelDelta) / 2.0).rotateBy(currentHeading);

        // Update current pose.
        this.currentPose = new Pose(Pose.PoseType.ABSOLUTE, currentPose.position.add(positionDelta), currentHeading);
    }

    public Pose getCurrentPose()
    {
        return currentPose;
    }

    public String[] getTrackingSummary()
    {
        return new String[]{
                "Pose is : " + getCurrentPose().toString()
        };
    }
}
