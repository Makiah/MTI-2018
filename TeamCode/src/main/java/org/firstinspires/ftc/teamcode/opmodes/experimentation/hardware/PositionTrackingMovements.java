package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;

@TeleOp(name="Position Tracking Movements", group= OpModeDisplayGroups.EXPERIMENTATION)
public class PositionTrackingMovements extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);

        robot.drivetrain.matchPose(
                robot.ptews,
                new Pose(Pose.PoseType.RELATIVE, new CartesianVector(10, 0), new DegreeAngle(0)),
                5,
                new DegreeAngle(10),
                null,
                flow);
    }
}
