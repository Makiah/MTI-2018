package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;
import hankutanku.music.Tunes;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;

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
        // Robot
        Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS, flow);

        // Vision
        VuforiaCam vuforiaCam = new VuforiaCam();

        // Set physical balance plate offset.
        Vector balancePlateOffset = new CartesianVector(24, 24); // starts at blue bottom
        Angle currentHeading = new DegreeAngle(0);
        if (getBalancePlate() == BalancePlate.TOP)
            balancePlateOffset.add(new CartesianVector(0, 72));
        if (getAlliance() == Alliance.RED)
        {
            balancePlateOffset.add(new CartesianVector(96, 0));
            currentHeading = new DegreeAngle(180);
        }
        log.lines("Starting position is " + balancePlateOffset.toString(false));

        robot.ptews.provideExternalPoseInformation(new Pose(Pose.PoseType.ABSOLUTE, balancePlateOffset, currentHeading));


        // region Initialization Detection of the Crypto Key
        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;
        vuforiaCam.start(true);
        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();

        long timeoutTimerStart = -1;

        while (true)
        {
            if (isStarted())
            {
                if (vumark != RelicRecoveryVuMark.UNKNOWN)
                    break;

                if (timeoutTimerStart < 0)
                    timeoutTimerStart = System.currentTimeMillis();

                if (System.currentTimeMillis() - timeoutTimerStart > 5000)
                    break;
            }

            RelicRecoveryVuMark newVumark = RelicRecoveryVuMark.from(relicTemplate);

            if (newVumark == RelicRecoveryVuMark.UNKNOWN)
            {
                flow.yield();
                continue;
            }

            if (vumark == RelicRecoveryVuMark.UNKNOWN || newVumark != vumark)
            {
                vumark = newVumark;

                switch (vumark)
                {
                    case LEFT:
                        Tunes.play(Tunes.Option.LEFT_COL, true);
                        break;
                    case CENTER:
                        Tunes.play(Tunes.Option.CENTER_COL, true);
                        break;
                    case RIGHT:
                        Tunes.play(Tunes.Option.RIGHT_COL, true);
                        break;
                }

                log.lines("Got new vumark: " + vumark.toString());
            }

            flow.yield();
        }

        vuforiaCam.stop(flow);

        // default vumark if none detected.
        if (vumark == RelicRecoveryVuMark.UNKNOWN)
            vumark = RelicRecoveryVuMark.CENTER;

        // endregion

        robot.jewelKnocker.knockJewel(getAlliance() == Alliance.BLUE, flow);

        // Steady drive (manual) off the balance board, using the rev hub gyros to determine when we're level (aka off the platform).
        robot.drivetrain.move(new CartesianVector(0, .25), 0);

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 500) // Give the robot some time to angle downward.
            flow.yield();

        while (Math.abs(robot.gyros.hubAnglesAveraged()[1].quickestDegreeMovementTo(new DegreeAngle(0))) > 5)
            flow.yield();

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            // Figure out the vector displacement to the correct deposit column (absolute)
            Vector depositVector = null;
            Angle depositAngle = null;

            switch (vumark) {
                case LEFT:
                    depositVector = new CartesianVector(4, 52); // offset by 4 from the 3rd tile beginning
                    break;

                case CENTER:
                    depositVector = new CartesianVector(4, 60);
                    break;

                case RIGHT:
                    depositVector = new CartesianVector(4, 68); // offset by -4 from the 3rd tile end
                    break;
            }

            switch (getAlliance())
            {
                case RED:
                    depositAngle = new DegreeAngle(90);
                    break;

                case BLUE:
                    depositAngle = new DegreeAngle(270);
                    break;
            }

            robot.drivetrain.matchPose(
                    "Deposit at Cryptobox",
                    robot.ptews,
                    new Pose(Pose.PoseType.ABSOLUTE, depositVector, depositAngle),
                    2,
                    new DegreeAngle(6),
                    null,
                    flow);

            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();
        }

        while (true)
            flow.yield();
    }
}
