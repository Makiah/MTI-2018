package org.firstinspires.ftc.teamcode.robot.hardware;

import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

public class PoseTrackingEncoderWheelSystem
{
    private final AbsoluteEncoder leftWheel, centerWheel, rightWheel; // These COULD be incremental, but we'd just be recalculating stuff.
    private Pose currentPose = new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(0, 0), new DegreeAngle(0)); // Reference changes over time.

    private Angle leftWheelPrevious, centerWheelPrevious, rightWheelPrevious;
    private double leftWheelCumulative = 0.0, rightWheelCumulative = 0.0, headingDegreeOffset = 0.0;

    private final double robotSpinCircumference = 13.4 * Math.PI; // width from left to right tracker.
    private final double angularToPositionalConversionConstant = 1 / 360.0 * (Math.PI * 4);

    private final ProcessConsole console;

    public PoseTrackingEncoderWheelSystem(AbsoluteEncoder leftWheel, AbsoluteEncoder centerWheel, AbsoluteEncoder rightWheel)
    {
        this.leftWheel = leftWheel;
        this.leftWheel.setDirection(AbsoluteEncoder.Direction.REVERSE);
        this.centerWheel = centerWheel;
        this.rightWheel = rightWheel;

        leftWheelPrevious = leftWheel.heading();
        centerWheelPrevious = centerWheel.heading();
        rightWheelPrevious = rightWheel.heading();

        console = LoggingBase.instance.newProcessConsole("PTEWS");
    }

    public void reset()
    {
        leftWheelPrevious = leftWheel.heading();
        centerWheelPrevious = centerWheel.heading();
        rightWheelPrevious = rightWheel.heading();

        leftWheelCumulative = 0.0;
        rightWheelCumulative = 0.0;
        headingDegreeOffset = 0.0;
    }

    public void provideExternalPoseInformation(Pose info)
    {
        if (info.poseType == Pose.PoseType.ABSOLUTE)
        {
            this.currentPose = info;
            headingDegreeOffset = info.heading.degrees();
        }
        else
        {
            this.currentPose.add(info);
            headingDegreeOffset += info.heading.degrees();
        }
    }

    public void update()
    {
        Angle leftWheelCurrent = leftWheel.heading(),
                centerWheelCurrent = centerWheel.heading(),
                rightWheelCurrent = rightWheel.heading();

        double leftWheelDelta = leftWheelPrevious.quickestDegreeMovementTo(leftWheelCurrent) * angularToPositionalConversionConstant,
                centerWheelDelta = centerWheelPrevious.quickestDegreeMovementTo(centerWheelCurrent) * angularToPositionalConversionConstant,
                rightWheelDelta = rightWheelPrevious.quickestDegreeMovementTo(rightWheelCurrent) * angularToPositionalConversionConstant;

        leftWheelPrevious = leftWheelCurrent;
        centerWheelPrevious = centerWheelCurrent;
        rightWheelPrevious = rightWheelCurrent;

        // The extent to which the left and right wheels have different measurements, represents angular orientation difference.
        // Doesn't at all depend on previous measurements.
        leftWheelCumulative += leftWheelDelta;
        rightWheelCumulative += rightWheelDelta;
        Angle currentHeading = new DegreeAngle((rightWheelCumulative - leftWheelCumulative) / robotSpinCircumference * 180.0 + headingDegreeOffset);

        // Now for position, which does in fact depend on previous movements.
        Vector positionDelta = new CartesianVector(centerWheelDelta, (rightWheelDelta + leftWheelDelta) / 2.0).rotateBy(currentHeading);

        // Update current pose.
        this.currentPose = new Pose(Pose.PoseType.ABSOLUTE, currentPose.position.add(positionDelta), currentHeading);

        console.write("Current pose is " + currentPose.toString());
    }

    public Pose getCurrentPose()
    {
        return currentPose;
    }
}
