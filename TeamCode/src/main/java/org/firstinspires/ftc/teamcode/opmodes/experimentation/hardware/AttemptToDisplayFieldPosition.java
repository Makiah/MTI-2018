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

@Autonomous(name="Attempt to Display FPosition", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class AttemptToDisplayFieldPosition extends EnhancedOpMode
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
        while (true)
        {
            Angle offset = new DegreeAngle(Math.random() * 360);

            // one tile
            drawRobotAtPose(new Pose(new CartesianVector(24, 24), offset), "#FFFF");
            drawRobotAtPose(new Pose(new CartesianVector(120, 24), offset), "#3F51B5");
            drawRobotAtPose(new Pose(new CartesianVector(120, 96), offset), "#3F51B5");
            drawRobotAtPose(new Pose(new CartesianVector(24, 96), offset), "#3F51B5");
        }
    }
}
