package org.firstinspires.ftc.teamcode.robot.structs;

import hankutanku.math.angle.Angle;
import hankutanku.math.vector.Vector;

/**
 * Expresses the field position and heading of the robot relative.
 */
public class Pose
{
    public enum PoseType { ABSOLUTE, RELATIVE }

    public final PoseType poseType;
    public final Vector position;
    public final Angle heading;

    public Pose(PoseType poseType, Vector position, Angle heading)
    {
        this.poseType = poseType;
        this.position = position;
        this.heading = heading;
    }

    public Pose add(Pose other)
    {
        return new Pose(PoseType.ABSOLUTE, position.add(other.position), heading.add(other.heading));
    }

    public Pose subtract(Pose other)
    {
        return new Pose(PoseType.ABSOLUTE, position.subtract(other.position), heading.subtract(other.heading));
    }

    public boolean acceptableMatch(Pose other, double minPositionDivergence, Angle maxAngleDivergence)
    {
        if (other.poseType == PoseType.RELATIVE)
        {
            return other.position.magnitude() <= minPositionDivergence && other.heading.degrees() < maxAngleDivergence.degrees();
        }
        else
        {
            return this.position.subtract(other.position).magnitude() <= minPositionDivergence && Math.abs(this.heading.quickestDegreeMovementTo(other.heading)) < maxAngleDivergence.degrees();
        }
    }

    public String toString()
    {
        return "Position is " + position.toString(false) + " and robot facing heading " + heading.degrees();
    }
}
