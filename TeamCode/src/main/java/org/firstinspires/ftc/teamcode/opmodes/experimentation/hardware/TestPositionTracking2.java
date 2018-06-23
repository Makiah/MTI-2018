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
{/**
 * Tells the mecanum drivetrain to match a given robot position and heading.
 * @param minimumInchesFromTarget  The maximum number of inches from the target pose which enables the robot to stop this code block.
 * @param minimumHeadingFromTarget  The maximum angle from the target pose which enables the robot to stop this code block.
 * @param timeoutLength  The maximum angle from the target pose which enables the robot to stop this code block.
 * @param completionBasedFunction  A function which returns void and receives a 0-1 input representing how close we are from execution completion.
 */
public void matchPTEWSDesiredPose(double minimumInchesFromTarget,
                                  Angle minimumHeadingFromTarget,
                                  TimeMeasure timeoutLength,
                                  Function<Void, Double> completionBasedFunction) throws InterruptedException
{
    ProcessConsole console = log.newProcessConsole("Match Pose");

    // Relative means relative to the current robot pose.
    Vector previousPositionalOffset = null;
    double previousHeadingOffset = Double.NaN;

    double movementPowerUpFactor = 0;
    double headingPowerUpFactor = 0;

    double originalTargetOffset = Double.NaN;

    long start = System.currentTimeMillis();

    while (true)
    {
        if (System.currentTimeMillis() - start > timeoutLength.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            LoggingBase.instance.lines("Timed out");
            break;
        }

        robot.ptews.update();

        Pose offset = robot.ptews.getPoseToDesired();

        Pose currentPose = robot.ptews.getCurrentPose();
        if (Double.isNaN(originalTargetOffset))
            originalTargetOffset = offset.position.magnitude();

        double headingDegreeOffsetFromTarget = offset.heading.quickestDegreeMovementTo(currentPose.heading);
        if (offset.position.magnitude() <= minimumInchesFromTarget && Math.abs(headingDegreeOffsetFromTarget) <= minimumHeadingFromTarget.degrees())
        {
            LoggingBase.instance.lines("Quit movement because position offset is " + offset.position.toString(false) + " and heading offset is " + offset.heading.degrees());
            break;
        }

        Vector move = offset.position.rotateBy(currentPose.heading.negative()).divide(20);
        robot.drivetrain.move(new PolarVector(Range.clip(move.magnitude(), -AutonomousSettings.maxMoveSpeed, AutonomousSettings.maxMoveSpeed), move.angle()), Range.clip(headingDegreeOffsetFromTarget / 20, -AutonomousSettings.maxTurnSpeed, AutonomousSettings.maxTurnSpeed));

        if (previousPositionalOffset != null)
        {
            Vector movedSinceLastLoop = offset.position.subtract(previousPositionalOffset);
            movementPowerUpFactor += (.03 - (movedSinceLastLoop.magnitude() / previousPositionalOffset.magnitude())) * .5; // should have moved 10 percent more of the way toward our target destination.
        }
        previousPositionalOffset = offset.position;

        if (!Double.isNaN(previousHeadingOffset))
        {
            double degreesMovedSinceLastLoop = new DegreeAngle(headingDegreeOffsetFromTarget).quickestDegreeMovementTo(new DegreeAngle(previousHeadingOffset));
            headingPowerUpFactor += (.03 - (Math.abs(degreesMovedSinceLastLoop) / Math.abs(previousHeadingOffset))) * .06; // should have moved 10 percent more of the required heading.
        }
        previousHeadingOffset = headingDegreeOffsetFromTarget;

        if (completionBasedFunction != null)
            completionBasedFunction.value((1.0 - offset.position.magnitude()) / originalTargetOffset);

        console.write(
                "Target pose is " + robot.ptews.getCurrentPose().toString(),
                "Positional offset is " + offset.position.toString(false),
                "Heading offset is " + Vector.decimalFormat.format(headingDegreeOffsetFromTarget),
                "Movement power up is " + movementPowerUpFactor,
                "Heading power up is " + headingPowerUpFactor);

        flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 30));
    }

    robot.drivetrain.stop();

    console.destroy();
}

    private Robot robot;

    @Override
    protected void onRun() throws InterruptedException
    {
        robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS, flow);

        robot.ptews.setCurrentPose(new Pose(new CartesianVector(72, 48), new DegreeAngle(0)));

        while (true)
        {
            Vector newVector = new CartesianVector(Math.random() * 144.0, Math.random() * 144.0);

            robot.ptews.setDesiredPose(new Pose(newVector, new DegreeAngle(0)));

            matchPTEWSDesiredPose(
                    5,
                    new DegreeAngle(5),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 100),
                    null);

            LoggingBase.instance.lines("New position " + newVector.toString(false));

            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 20));
        }
    }
}
