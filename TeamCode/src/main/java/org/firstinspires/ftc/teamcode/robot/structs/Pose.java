package org.firstinspires.ftc.teamcode.robot.structs;

import java.text.DecimalFormat;

import hankutanku.math.angle.Angle;
import hankutanku.math.vector.Vector;

/**
 * Expresses the field position and heading of the robot relative.
 */
public class Pose
{
    private static final DecimalFormat decimalFormat = new DecimalFormat("#.00");

    public final Vector position;
    public final Angle heading;

    public Pose(Vector position, Angle heading)
    {
        this.position = position;
        this.heading = heading;
    }

    public Pose add(Pose other)
    {
        return new Pose(position.add(other.position), heading.add(other.heading));
    }

    public Pose add(Vector position)
    {
        return new Pose(this.position.add(position), heading);
    }

    public Pose add(Angle heading)
    {
        return new Pose(position, this.heading.add(heading));
    }

    public Pose subtract(Pose other)
    {
        return new Pose(position.subtract(other.position), heading.subtract(other.heading));
    }

    public boolean acceptableMatch(Pose other, double minPositionDivergence, Angle maxAngleDivergence)
    {
        return this.position.subtract(other.position).magnitude() <= minPositionDivergence && Math.abs(this.heading.quickestDegreeMovementTo(other.heading)) < maxAngleDivergence.degrees();
    }

    public String toString()
    {
        return "position " + position.toString(false) + ", heading " + decimalFormat.format(heading.degrees());
    }
}
