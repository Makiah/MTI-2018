package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.PolarVector;

@Autonomous(name="Test Timed Drives", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestTimedDrives extends EnhancedOpMode
{
    private Robot robot;

    @Override
    protected void onRun() throws InterruptedException
    {
        robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS, flow);

        robot.ptews.setCurrentPose(new Pose(new CartesianVector(72, 48), new DegreeAngle(0)));

        robot.drivetrain.turnToHeading(robot.ptews, new DegreeAngle(270), 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 2), flow);

        robot.drivetrain.driveTime(robot.ptews, new PolarVector(.4, new DegreeAngle(270)), new DegreeAngle(270), new TimeMeasure(TimeMeasure.Units.SECONDS, 2), flow);

        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));

        robot.drivetrain.turnToHeading(robot.ptews, new DegreeAngle(0), 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 2), flow);

        robot.drivetrain.driveTime(robot.ptews, new PolarVector(.4, new DegreeAngle(0)), new DegreeAngle(0), new TimeMeasure(TimeMeasure.Units.SECONDS, 2), flow);

        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));
    }
}
