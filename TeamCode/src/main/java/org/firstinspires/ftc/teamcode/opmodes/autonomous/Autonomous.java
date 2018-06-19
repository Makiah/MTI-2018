package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

/**
 * FIELD MAP
 *
 * (0, 144)   (0 degrees)         (144, 144)
 *  ______________________________
 * |    c        TOP         c    |
 * |                              |
 * |  b                        b  |
 * |                              |
 * |                              |
 * | BLUE                         |   (270 degrees)
 * | c                          c |
 * |                          RED |
 * |                              |
 * |                              |
 * |  b         BOTTOM         b  |
 * |______________________________|
 * (0, 0)      (180 degrees)       (144, 0)
 */
public abstract class Autonomous extends EnhancedOpMode implements CompetitionProgram
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware);

        Vector balancePlateOffset = new CartesianVector(24, 24); // starts at blue bottom
        Angle currentHeading = new DegreeAngle(270);
        if (getBalancePlate() == BalancePlate.TOP)
            balancePlateOffset.add(new CartesianVector(0, 72));
        if (getAlliance() == Alliance.RED)
        {
            balancePlateOffset.add(new CartesianVector(96, 0));
            currentHeading = new DegreeAngle(90);
        }

        robot.ptews.provideExternalPoseInformation(new Pose(Pose.PoseType.ABSOLUTE, balancePlateOffset, currentHeading));
    }
}
