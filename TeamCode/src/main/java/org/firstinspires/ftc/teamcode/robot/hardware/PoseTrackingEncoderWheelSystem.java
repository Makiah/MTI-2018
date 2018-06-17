package org.firstinspires.ftc.teamcode.robot.hardware;

import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

public class PoseTrackingEncoderWheelSystem
{
    private final IncrementalAbsoluteEncoder leftWheel, centerWheel, rightWheel;
    private Pose currentPose = new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(0, 0), new DegreeAngle(0)); // Reference changes over time.
    private double leftWheelPrevious = 0.0, centerWheelPrevious = 0.0, rightWheelPrevious = 0.0;

    private final double robotSpinCircumference = 18 * Math.PI;
    private final double angularToPositionalConversionConstant = 1 / 360.0 * (Math.PI * 4);

    public PoseTrackingEncoderWheelSystem(IncrementalAbsoluteEncoder leftWheel, IncrementalAbsoluteEncoder centerWheel, IncrementalAbsoluteEncoder rightWheel)
    {
        this.leftWheel = leftWheel;
        this.centerWheel = centerWheel;
        this.rightWheel = rightWheel;
    }

    public void update()
    {
        leftWheel.updateIncremental();
        centerWheel.updateIncremental();
        rightWheel.updateIncremental();

        double leftWheelPosition = leftWheel.getTotalAngularOffset() * angularToPositionalConversionConstant,
                centerWheelPosition = centerWheel.getTotalAngularOffset() * angularToPositionalConversionConstant,
                rightWheelPosition = rightWheel.getTotalAngularOffset() * angularToPositionalConversionConstant;

        double leftWheelDelta = leftWheelPosition - leftWheelPrevious,
                centerWheelDelta = centerWheelPosition - centerWheelPrevious,
                rightWheelDelta = rightWheelPosition - rightWheelPrevious;

        leftWheelPrevious = leftWheelPosition;
        centerWheelPrevious = centerWheelPosition;
        rightWheelPrevious = rightWheelPosition;

        // The extent to which the left and right wheels have different measurements, represents angular orientation difference.
        // Doesn't at all depend on previous measurements.
        Angle currentHeading = new DegreeAngle((rightWheelPosition - leftWheelPosition) / robotSpinCircumference * 180.0);

        // Now for position, which does in fact depend on previous movements.
        Vector positionDelta = new CartesianVector(centerWheelDelta, (rightWheelDelta + leftWheelDelta) / 2.0).rotateBy(currentHeading);

        // Update current pose.
        this.currentPose = new Pose(Pose.PoseType.ABSOLUTE, currentPose.position.add(positionDelta), currentHeading);
    }

    public Pose getCurrentPose()
    {
        return currentPose;
    }
}
