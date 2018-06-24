package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousSettings;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.function.Function;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;

@Autonomous(name="Test Pose Tracking 2", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestPositionTracking2 extends EnhancedOpMode
{
    private Robot robot;

    @Override
    protected void onRun() throws InterruptedException
    {
        robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS, flow);

        robot.ptews.setCurrentPose(new Pose(new CartesianVector(72, 48), new DegreeAngle(0)));

        while (true)
        {
            robot.ptews.setDesiredPose(new Pose(new CartesianVector(Math.random() * 144.0, Math.random() * 144.0), new DegreeAngle(Math.random() * 360)));

            robot.drivetrain.matchPTEWSDesiredPose(
                    robot.ptews,
                    5,
                    new DegreeAngle(5),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 100),
                    null,
                    flow);

            LoggingBase.instance.lines("New position " + robot.ptews.getDesiredPose().position.toString(false));

            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 20));
        }
    }
}
