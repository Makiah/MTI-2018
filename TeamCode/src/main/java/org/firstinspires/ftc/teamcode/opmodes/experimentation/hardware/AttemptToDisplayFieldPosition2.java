package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.acmedashboard.DrawingUtil;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;

@Autonomous(name="Attempt to Display Auto", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class AttemptToDisplayFieldPosition2 extends EnhancedOpMode
{
    public void drawRobotAtPose(Pose pose, String stroke) throws InterruptedException
    {
        TelemetryPacket tPacket = new TelemetryPacket();
        Canvas fieldOverlay = tPacket.fieldOverlay();

        fieldOverlay.setStroke(stroke);
        DrawingUtil.drawMecanumRobot(fieldOverlay, pose);

        RobotDashboard.getInstance().sendTelemetryPacket(tPacket);

        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        long start = System.currentTimeMillis();
        long time = 10000;
        while (System.currentTimeMillis() - start < time)
        {
            double x = (1.0 * System.currentTimeMillis() - start) / time;

            // one tile
            drawRobotAtPose(new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(24, 24 + x * 24), new DegreeAngle(0)), "#FFFF");

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
        }

        start = System.currentTimeMillis();
        time = 10000;
        while (System.currentTimeMillis() - start < time)
        {
            double x = (1.0 * System.currentTimeMillis() - start) / time;

            // one tile
            drawRobotAtPose(new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(24 + x * 2, 24 + x * 18), new DegreeAngle(-90 * x)), "#FFFF");

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
        }

        start = System.currentTimeMillis();
        time = 5000;
        while (System.currentTimeMillis() - start < time)
        {
            double x = (1.0 * System.currentTimeMillis() - start) / time;

            // one tile
            drawRobotAtPose(new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(24 + x * -8, 42), new DegreeAngle(-90)), "#FFFF");

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
        }
    }
}
