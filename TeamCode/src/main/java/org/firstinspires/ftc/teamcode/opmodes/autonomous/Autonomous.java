package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector2;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.PolarVector;
import hankutanku.math.vector.Vector;
import hankutanku.music.Tunes;
import hankutanku.phonesensors.AndroidGyro;
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
//@Config //(doesn't work on abstract classes)
public abstract class Autonomous extends EnhancedOpMode implements CompetitionProgram
{
    private Robot robot;
    private Cryptobox cryptobox;

    /**
     * Harvests glyphs in a state-machine type of thing.
     * @throws InterruptedException
     * @return the number of glyphs harvested (hopefully two).
     */
    private int harvestGlyphs() throws InterruptedException
    {
        robot.harvester.run(-AutonomousSettings.harvestPower);
        int glyphsGrabbed = 0;

        robot.drivetrain.move(new CartesianVector(0, AutonomousSettings.harvestMoveForwardSpeed), 0);
        while (robot.ptews.getCurrentPose().position.x() < 60)
        {
            robot.ptews.update();

            if (glyphsGrabbed == 0)
            {
                if (robot.glyphSensor.getDistance(DistanceUnit.CM) < 13)
                {
                    glyphsGrabbed++;
                    Tunes.play(Tunes.Option.ONE_GLYPH, true);
                }
            }
            else if (glyphsGrabbed == 1)
            {
                if (robot.glyphSensor2.getDistance(DistanceUnit.CM) < 15)
                {
                    glyphsGrabbed++;
                    Tunes.play(Tunes.Option.TWO_GLYPH, true);
                    break;
                }
            }

            flow.yield();
        }

        robot.harvester.run(0);

        return glyphsGrabbed;
    }

    /**
     * Dump glyphs into the cryptobox using tracking wheels
     * @param col the column to dump in
     * @param numGlyphs the number of glyphs to dump
     * @throws InterruptedException
     */
    private void dump(Cryptobox.Column col, int numGlyphs) throws InterruptedException
    {
        robot.flipper.attemptFlipperStateIncrement();
        while (robot.flipper.isMidStateTransition())
            robot.flipper.updateFlipperState();

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Figure out the vector displacement to the correct deposit column (absolute)
            Vector absoluteDepositLocation = null;
            Angle absoluteDepositAngle = null;

            Angle glyphDepositAngle = new DegreeAngle(0);
            Angle columnOffset = new DegreeAngle(0);

            final double cryptoboxXPos = getAlliance() == Alliance.BLUE ? 24 : 120;

            switch (col) {
                case LEFT:
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, 52); // offset by 4 from the 3rd tile beginning
                    columnOffset = new DegreeAngle(AutonomousSettings.depositAngleOffset);
                    break;

                case CENTER:
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, 60);
                    columnOffset = new DegreeAngle(-AutonomousSettings.depositAngleOffset);
                    break;

                case RIGHT:
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, 68); // offset by -4 from the 3rd tile end
                    columnOffset = new DegreeAngle(-AutonomousSettings.depositAngleOffset);
                    break;
            }

            switch (getAlliance()) {
                case RED:
                    columnOffset = columnOffset.negative();
                    glyphDepositAngle = new DegreeAngle(90);
                    absoluteDepositAngle = glyphDepositAngle.add(columnOffset);
                    break;

                case BLUE:
                    glyphDepositAngle = new DegreeAngle(270);
                    absoluteDepositAngle = glyphDepositAngle.add(columnOffset);
                    break;
            }

            Vector vectorOffsetFromColumnAngleOffset = new PolarVector(5, glyphDepositAngle.add(columnOffset));

            robot.drivetrain.matchPose(
                    "In Front of Column",
                    robot.ptews,
                    new Pose(Pose.PoseType.ABSOLUTE, absoluteDepositLocation.add(vectorOffsetFromColumnAngleOffset), absoluteDepositAngle),
                    5,
                    new DegreeAngle(3),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 100),
                    null,
                    flow);

            // Flip out
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            // Register the new glyphs
            cryptobox.dump(col, numGlyphs);

            // Push it in
            robot.drivetrain.move(new CartesianVector(0, -.4), col != Cryptobox.Column.LEFT ? -.3 : .3);
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 600)
            {
                robot.ptews.update();
                flow.yield();
            }
            robot.drivetrain.move(null, 0);

            // Drive away
            switch (col)
            {
                case LEFT:
                    robot.drivetrain.move(new CartesianVector(.2, .4), 0);
                    break;

                case CENTER:
                    robot.drivetrain.move(new CartesianVector(0, .5), 0);
                    break;

                case RIGHT:
                    robot.drivetrain.move(new CartesianVector(-.2, .4), 0);
                    break;
            }
            start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 600)
            {
                robot.ptews.update();
                flow.yield();
            }
            robot.drivetrain.move(null, 0);

            // Flip back
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            robot.drivetrain.matchPose(
                    "Head toward pile",
                    robot.ptews,
                    new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(24, 50), getAlliance() == Alliance.BLUE ? new DegreeAngle(270) : new DegreeAngle(90)),
                    3,
                    new DegreeAngle(4),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 100),
                    null,
                    flow);
        }
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        // Robot
        robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS, flow);

        // Cryptobox
        cryptobox = new Cryptobox();

        // Gyro
        AndroidGyro phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();
        phoneGyro.initAntiDrift();

        // Vision
        OpenCVCam openCVCam = new OpenCVCam();
        VuforiaCam vuforiaCam = new VuforiaCam();
        JewelDetector2 jewelDetector = new JewelDetector2();

        // Set physical balance plate offset.
        Vector balancePlateOffset = new CartesianVector(24, 24); // starts at blue bottom
        Angle balancePlateHeading = new DegreeAngle(0);
        if (getBalancePlate() == BalancePlate.TOP)
            balancePlateOffset.add(new CartesianVector(0, 72));
        if (getAlliance() == Alliance.RED)
        {
            balancePlateOffset.add(new CartesianVector(96, 0));
            balancePlateHeading = new DegreeAngle(180);
        }
        log.lines("Starting position is " + balancePlateOffset.toString(false));

        // region Initialization Detection of the Crypto Key and Jewel Alignment

        // DON'T specify a default order, if we mess this up we lose points.
        JewelDetector2.JewelOrder jewelOrder = JewelDetector2.JewelOrder.UNKNOWN;
        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;

        // Loop through
        ProcessConsole observedConsole = log.newProcessConsole("Observed Init stuff");
        while (!isStarted()) // Runs until OpMode is started, then just goes from there.
        {
            // Jewel detection
            openCVCam.start(jewelDetector);
            JewelDetector2.JewelOrder newJewelOrder = JewelDetector2.JewelOrder.UNKNOWN;
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 5000 && newJewelOrder == JewelDetector2.JewelOrder.UNKNOWN)
            {
                newJewelOrder = jewelDetector.getCurrentOrder();

                if (isStarted() && jewelOrder != JewelDetector2.JewelOrder.UNKNOWN)
                    break;

                flow.yield();
            }
            openCVCam.stop();

            if (newJewelOrder != JewelDetector2.JewelOrder.UNKNOWN)
            {
                if (jewelOrder != newJewelOrder)
                {
                    // TODO recreate audio
//                    switch (newJewelOrder)
//                    {
//                        case BLUE_RED:
//                            Tunes.play(getAlliance() == Alliance.RED ? Tunes.Option.LEFT_RIGHT_JEWEL : Tunes.Option.RIGHT_LEFT_JEWEL);
//                            break;
//
//                        case RED_BLUE:
//                            Tunes.play(getAlliance() == Alliance.RED ? Tunes.Option.RIGHT_LEFT_JEWEL : Tunes.Option.LEFT_RIGHT_JEWEL);
//                            break;
//                    }
//
//                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
                }

                jewelOrder = newJewelOrder;
            }

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            start = System.currentTimeMillis();

            if (isStarted() && vumark != RelicRecoveryVuMark.UNKNOWN)
                break;

            // VuMark detection.
            RelicRecoveryVuMark newVuMark = RelicRecoveryVuMark.UNKNOWN;
            vuforiaCam.start(true);
            VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
            vuforiaCam.getTrackables().activate();
            while (System.currentTimeMillis() - start < 10000 && newVuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                newVuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (isStarted() && vumark != RelicRecoveryVuMark.UNKNOWN)
                    break;

                flow.yield();
            }

            vuforiaCam.stop(flow);

            if (newVuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                if (vumark != newVuMark)
                {
                    switch (newVuMark)
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

                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));
                }

                vumark = newVuMark;
            }

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            flow.yield();
        }
        observedConsole.destroy();

        // default vumark if none detected.
        if (vumark == RelicRecoveryVuMark.UNKNOWN)
            vumark = RelicRecoveryVuMark.CENTER;
        // endregion

        // Start anti-drift
        phoneGyro.startAntiDrift();

        // region Knock Ball
        if (jewelOrder != JewelDetector2.JewelOrder.UNKNOWN)
        {
            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (jewelOrder == JewelDetector2.JewelOrder.BLUE_RED)
                    robot.jewelKnocker.knockJewel(false, flow);
                else
                    robot.jewelKnocker.knockJewel(true, flow);

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (jewelOrder == JewelDetector2.JewelOrder.BLUE_RED)
                    robot.jewelKnocker.knockJewel(true, flow);
                else
                    robot.jewelKnocker.knockJewel(false, flow);
            }
        }
        // endregion

        robot.relic.setRotatorPosition(true);

        robot.harvester.run(-.6);

        // Steady drive (manual) off the balance board, using the rev hub gyros to determine when we're level (aka off the platform).
        robot.drivetrain.move(new CartesianVector(0, .2), 0);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 850) // Give the robot some time to angle downward.
            flow.yield();
        while (true)
        {
            double balanceBoardFrontTilt = phoneGyro.z;
            if (balanceBoardFrontTilt < .1)
                break;
            if (System.currentTimeMillis() - start > 2000) // timeout failsafe
            {
                LoggingBase.instance.lines("Timed out, exiting.");
                break;
            }
            flow.yield();
        }
        robot.drivetrain.move(null, 0);
        robot.harvester.run(0);

        robot.ptews.reset();
        robot.ptews.provideExternalPoseInformation(new Pose(Pose.PoseType.ABSOLUTE, balancePlateOffset.add(new CartesianVector(0, 22)), balancePlateHeading)); // untracked distance.

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            switch (vumark)
            {
                case LEFT:
                    dump(Cryptobox.Column.LEFT, 1);
                    break;

                case CENTER:
                    dump(Cryptobox.Column.CENTER, 1);
                    break;

                case RIGHT:
                    dump(Cryptobox.Column.RIGHT, 1);
                    break;
            }

            while (true)
            {
                dump(cryptobox.getOptimalDumpLocation(), harvestGlyphs());
            }
        }

        while (true)
            flow.yield();
    }
}
