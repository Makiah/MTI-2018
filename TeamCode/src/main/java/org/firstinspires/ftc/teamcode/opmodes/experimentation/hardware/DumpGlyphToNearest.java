package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.CryptoboxTracker;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Dump Glyph to Nearest", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DumpGlyphToNearest extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        CryptoboxTracker tracker = new CryptoboxTracker();
        tracker.setAlliance(CompetitionProgram.Alliance.BLUE);
        tracker.setTrackingMode(CryptoboxTracker.ColumnTrackingMode.SIMPLE);
        tracker.setLoggingEnabledTo(true);

        OpenCVCam cam = new OpenCVCam();
        cam.setMaxResolution(800, 500);
        cam.start(tracker, true);

        robot.lights.setLightsTo(true);

        waitForStart();

        // Tell the gyro to start anti-drift stuff.
        robot.gyro.startAntiDrift();

        double offFromForwardIdeal = -(tracker.estimatedForwardDistance - .26), offFromHorizontalIdeal = -tracker.closestPlacementLocationOffset;

        // Double equality
        while (Math.abs(offFromForwardIdeal) > 0.05 || Math.abs(offFromHorizontalIdeal) > 0.04)
        {
            double horizontalSpeed, forwardSpeed;
            if (!tracker.detectedNoColumns)
            {
                forwardSpeed = Math.signum(offFromForwardIdeal) * (Math.abs(offFromForwardIdeal) * 0 + .1);
                if (Math.abs(forwardSpeed) < .015)
                    forwardSpeed = 0;

                horizontalSpeed = Math.signum(offFromHorizontalIdeal) * (Math.abs(offFromHorizontalIdeal) * 0 + .04);
                if (Math.abs(horizontalSpeed) < .015)
                    horizontalSpeed = 0;
            }
            else
            {
                horizontalSpeed = 0;
                forwardSpeed = 0.1;
                offFromForwardIdeal = -1;
                offFromHorizontalIdeal = -1;
            }

            robot.swerveDrive.setDesiredMovement(Vector2D.rectangular(forwardSpeed, horizontalSpeed));

            // Wait until we have a new reading (frames can take a while to process)
            double currentOffFromForward = offFromForwardIdeal, currentOffFromHorizontal = offFromHorizontalIdeal; // use other vals as anchor points
            while (Math.abs(currentOffFromForward - offFromForwardIdeal) < .00001 && Math.abs(currentOffFromHorizontal - offFromHorizontalIdeal) < .00001)
            {
                currentOffFromForward = -(tracker.estimatedForwardDistance - .26);
                currentOffFromHorizontal = -tracker.closestPlacementLocationOffset;

                robot.swerveDrive.synchronousUpdate();

                flow.yield();
            }

            // Start a new loop with these anchor points.
            offFromForwardIdeal = currentOffFromForward;
            offFromHorizontalIdeal = currentOffFromHorizontal;
        }

        robot.swerveDrive.stop();

        robot.intake.intake();

        flow.msPause(1000);

        robot.flipper.advanceStage(2);

        flow.msPause(1000);
    }
}
