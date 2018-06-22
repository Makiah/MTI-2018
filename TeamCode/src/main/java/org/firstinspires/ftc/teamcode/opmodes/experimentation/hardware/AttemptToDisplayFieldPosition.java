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
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;

@Autonomous(name="Attempt to Display FPosition", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class AttemptToDisplayFieldPosition extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        long start = System.currentTimeMillis();

        while (true)
        {
            TelemetryPacket tPacket = new TelemetryPacket();
            Canvas fieldOverlay = tPacket.fieldOverlay();

            fieldOverlay.setStroke("#3F51B5");
            DrawingUtil.drawMecanumRobot(fieldOverlay, new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(50, 50 + (System.currentTimeMillis() - start) / 1000), new DegreeAngle(45)));

            RobotDashboard.getInstance().sendTelemetryPacket(tPacket);

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
        }
    }
}
