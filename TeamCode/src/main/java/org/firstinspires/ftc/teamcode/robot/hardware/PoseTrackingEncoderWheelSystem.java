package org.firstinspires.ftc.teamcode.robot.hardware;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.acmedashboard.DrawingUtil;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

public class PoseTrackingEncoderWheelSystem
{
    private final IncrementalAbsoluteEncoder leftWheel, centerWheel, rightWheel; // These COULD be incremental, but we'd just be recalculating stuff.
    private Pose currentPose = new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(0, 0), new DegreeAngle(0)); // Reference changes over time.

    private double leftWheelCumulativePrevious = Double.NaN, centerWheelCumulativePrevious = Double.NaN, rightWheelCumulativePrevious = Double.NaN;

    private final double robotSpinCircumference = 13.27 * Math.PI; // width from left to right tracker.
    private final double angularToPositionalConversionConstant = 1 / 360.0 * (Math.PI * 4);

    private final ProcessConsole console;

    public PoseTrackingEncoderWheelSystem(AbsoluteEncoder leftWheel, AbsoluteEncoder centerWheel, AbsoluteEncoder rightWheel)
    {
        leftWheel.setDirection(AbsoluteEncoder.Direction.REVERSE);
        this.leftWheel = new IncrementalAbsoluteEncoder(leftWheel);
        this.centerWheel = new IncrementalAbsoluteEncoder(centerWheel);
        this.rightWheel = new IncrementalAbsoluteEncoder(rightWheel);

        console = LoggingBase.instance.newProcessConsole("PTEWS");
    }

    public void reset()
    {
        leftWheelCumulativePrevious = Double.NaN;
        centerWheelCumulativePrevious = Double.NaN;
        rightWheelCumulativePrevious = Double.NaN;

        leftWheel.resetEncoderWheel();
        centerWheel.resetEncoderWheel();
        rightWheel.resetEncoderWheel();

        this.currentPose = new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(0, 0), new DegreeAngle(0));
    }

    public void provideExternalPoseInformation(Pose info)
    {
        if (info.poseType == Pose.PoseType.ABSOLUTE)
        {
            this.currentPose = info;
        }
        else
        {
            this.currentPose.add(info);
        }
    }

    private void redrawRobotForDashboard()
    {
        if (RobotDashboard.getInstance() == null)
            return;

        LoggingBase.instance.lines("Drew Robot");

        TelemetryPacket tPacket = new TelemetryPacket();
        Canvas fieldOverlay = tPacket.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DrawingUtil.drawMecanumRobot(fieldOverlay, getCurrentPose());

        RobotDashboard.getInstance().sendTelemetryPacket(tPacket);
    }

    public void update()
    {
        leftWheel.updateIncremental();
        centerWheel.updateIncremental();
        rightWheel.updateIncremental();

        double leftWheelCumulativeCurrent = leftWheel.getTotalDegreeOffset(),
                centerWheelCumulativeCurrent = centerWheel.getTotalDegreeOffset(),
                rightWheelCumulativeCurrent = rightWheel.getTotalDegreeOffset();

        if (!Double.isNaN(leftWheelCumulativePrevious)) // hasn't been un-reset yet.
        {
            double leftWheelDelta = (leftWheelCumulativeCurrent - leftWheelCumulativePrevious) * angularToPositionalConversionConstant,
                    centerWheelDelta = (centerWheelCumulativeCurrent - centerWheelCumulativePrevious) * angularToPositionalConversionConstant,
                    rightWheelDelta = (rightWheelCumulativeCurrent - rightWheelCumulativePrevious) * angularToPositionalConversionConstant;

            // The extent to which the left and right wheels have different measurements, represents angular orientation difference.
            // Doesn't at all depend on previous measurements.

            // Now for position, which does in fact depend on previous movements.
            Vector positionDelta = new CartesianVector(centerWheelDelta, (rightWheelDelta + leftWheelDelta) / 2.0).rotateBy(this.currentPose.heading);

            Angle deltaAngle = new DegreeAngle((rightWheelDelta - leftWheelDelta) / robotSpinCircumference * 180.0);

            // Update current pose.
            this.currentPose = new Pose(Pose.PoseType.ABSOLUTE, currentPose.position.add(positionDelta), currentPose.heading.add(deltaAngle));

            console.write("Current pose is " + currentPose.toString());
        }

        leftWheelCumulativePrevious = leftWheelCumulativeCurrent;
        centerWheelCumulativePrevious = centerWheelCumulativeCurrent;
        rightWheelCumulativePrevious = rightWheelCumulativeCurrent;

        redrawRobotForDashboard();
    }

    public Pose getCurrentPose()
    {
        return currentPose;
    }
}
