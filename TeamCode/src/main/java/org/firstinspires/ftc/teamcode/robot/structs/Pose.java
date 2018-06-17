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

    public String toString()
    {
        return "Position is " + position.toString() + " and heading is " + heading.degrees();
    }
}
