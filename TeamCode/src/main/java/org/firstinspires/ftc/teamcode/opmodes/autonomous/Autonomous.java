package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.util.Range;

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
import hankutanku.math.function.Function;
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

    protected abstract boolean dontRunAuto();

    @Override
    protected void onRun() throws InterruptedException
    {
        // Update constants based on battery coefficient
        AutonomousSettings.harvestMoveForwardSpeed -= (batteryCoefficient) * .3;
        AutonomousSettings.maxMoveSpeed -= (batteryCoefficient) * .3;
        AutonomousSettings.maxTurnSpeed -= (batteryCoefficient) * .2;

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
            balancePlateOffset = balancePlateOffset.add(new CartesianVector(0, 72));
        if (getAlliance() == Alliance.RED)
        {
            balancePlateOffset = balancePlateOffset.add(new CartesianVector(96, 0));
            balancePlateHeading = new DegreeAngle(180);
        }
        log.lines("Starting position is " + balancePlateOffset.toString(false));
        robot.ptews.setCurrentPose(new Pose(balancePlateOffset, balancePlateHeading)); // untracked distance.

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

            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));

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
                }

                vumark = newVuMark;
            }

            observedConsole.write("Jewels: " + jewelOrder.toString(), "VuMark: " + vumark.toString());

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

        // Otherwise it gets in the way of the flipper
        robot.relic.setRotatorPosition(false);

        // So they sproing out
        robot.harvester.run(.6);

        if (dontRunAuto())
        {
            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
            return;
        }

        // Steady drive (manual) off the balance board, using the rev hub gyros to determine when we're level (aka off the platform).

        if (getAlliance() == Alliance.RED)
        {
            robot.drivetrain.move(new CartesianVector(0, getAlliance() == Alliance.RED ? -.4 : .2), 0);
            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 300));
        }

        robot.drivetrain.move(new CartesianVector(0, getAlliance() == Alliance.BLUE ? .2 : -.2), 0);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 750) // Give the robot some time to angle downward.
            flow.yield();
        while (true)
        {
            double balanceBoardFrontTilt = phoneGyro.z;
            boolean offBalanceBoard = false;

            if (getAlliance() == Alliance.BLUE)
                offBalanceBoard = balanceBoardFrontTilt < .1;
            else
                offBalanceBoard = balanceBoardFrontTilt > -.1;

            if (offBalanceBoard)
            {
                break;
            }
            if (System.currentTimeMillis() - start > 1000) // timeout failsafe
            {
                LoggingBase.instance.lines("Timed out, exiting.");
                break;
            }
            flow.yield();
        }
        robot.drivetrain.move(null, 0);
        robot.harvester.run(0);

        // Set the positioning system's offset
        robot.ptews.reset();
        robot.ptews.setCurrentPose(new Pose(balancePlateOffset.add(getAlliance() == Alliance.BLUE ? new CartesianVector(0, 28) : new CartesianVector(0, -22)), balancePlateHeading)); // untracked distance.

        // Actually take care of auto.
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

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Get ALL THE GLYPHS
            while (true)
                dump(cryptobox.getOptimalDumpLocation(), harvestGlyphs());
        }
    }

    /**
     * Drives the robot by some vector and turn speed for some length of time
     * @param driveVector  vector to drive
     * @param turnSpeed  turn speed
     * @param driveTime  time to drive
     * @param updatePTEWS  whether to update the tracker wheels
     * @throws InterruptedException
     */
    public void driveTime(Vector driveVector, double turnSpeed, TimeMeasure driveTime, boolean updatePTEWS) throws InterruptedException
    {
        robot.drivetrain.move(driveVector, turnSpeed);

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < driveTime.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            if (updatePTEWS)
                robot.ptews.update();

            flow.yield();
        }

        robot.drivetrain.stop();
    }
    /**
     * Overload of above.
     */
    public void driveTime(Vector driveVector, double turnSpeed, TimeMeasure driveTime) throws InterruptedException
    {
        driveTime(driveVector, turnSpeed, driveTime, true);
    }

    public void turnToHeading(Angle heading, double acceptableDifferenceDegrees, TimeMeasure timeout) throws InterruptedException
    {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeout.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            robot.drivetrain.move(null, -1 * Range.clip(robot.ptews.getCurrentPose().heading.quickestDegreeMovementTo(heading) / 20, -AutonomousSettings.maxTurnSpeed, AutonomousSettings.maxTurnSpeed));
            robot.ptews.update();

            if (Math.abs(robot.ptews.getCurrentPose().heading.quickestDegreeMovementTo(heading)) < acceptableDifferenceDegrees)
                break;

            flow.yield();
        }
        robot.drivetrain.stop();
    }

    /**
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

    /**
     * Harvests glyphs in a state-machine type of thing.
     * @throws InterruptedException
     * @return the number of glyphs harvested (hopefully two).
     */
    private int harvestGlyphs() throws InterruptedException
    {
//        if (System.currentTimeMillis() - autoStartTime > 25000)
//        {
//            matchPose(
//                    new CartesianVector(getAlliance() == Alliance.BLUE ? 32 : 112, 62));
//        }

        robot.ptews.setDesiredPose(
                new Pose(
                        new CartesianVector(getAlliance() == Alliance.BLUE ? 32 : 112, 62),
                        getAlliance() == Alliance.BLUE ? new DegreeAngle(270) : new DegreeAngle(90)));

        matchPTEWSDesiredPose(
                3,
                new DegreeAngle(4),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 4),
                null);

        robot.harvester.run(AutonomousSettings.harvestPower);
        int glyphsGrabbed = 0;

        robot.drivetrain.move(new CartesianVector(0, AutonomousSettings.harvestMoveForwardSpeed), 0);
        boolean notOnOpposingAlliance = true;
        while (notOnOpposingAlliance && glyphsGrabbed < 2)
        {
            if (getAlliance() == Alliance.BLUE)
                notOnOpposingAlliance = robot.ptews.getCurrentPose().position.x() < 56;
            else
                notOnOpposingAlliance = robot.ptews.getCurrentPose().position.x() > 72;

            robot.ptews.update();
            robot.harvester.update();

            if (glyphsGrabbed == 0)
            {
                if (robot.glyphSensor.getDistance(DistanceUnit.CM) < 13)
                {
                    glyphsGrabbed++;
                    Tunes.play(Tunes.Option.ONE_GLYPH, true);

                    driveTime(new CartesianVector(0, -AutonomousSettings.harvestMoveForwardSpeed / 2.0), 0, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 800));
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

        robot.drivetrain.stop();
        robot.harvester.run(0);

        robot.ptews.setDesiredPose(new Pose(new CartesianVector(getAlliance() == Alliance.BLUE ? 24 : 120, 62), getAlliance() == Alliance.BLUE ? new DegreeAngle(270) : new DegreeAngle(90)));
        matchPTEWSDesiredPose(
                3,
                new DegreeAngle(4),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 4),
                null);

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
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, AutonomousSettings.leftDrive); // offset by 4 from the 3rd tile beginning
                    columnOffset = new DegreeAngle(AutonomousSettings.depositAngleOffset);
                    break;

                case CENTER:
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, AutonomousSettings.centerDrive);
                    columnOffset = new DegreeAngle(-AutonomousSettings.depositAngleOffset);
                    break;

                case RIGHT:
                    absoluteDepositLocation = new CartesianVector(cryptoboxXPos, AutonomousSettings.rightDrive); // offset by -4 from the 3rd tile end
                    columnOffset = new DegreeAngle(-AutonomousSettings.depositAngleOffset);
                    break;
            }

            switch (getAlliance()) {
                case RED:
                    columnOffset = columnOffset.negative();
                    glyphDepositAngle = new DegreeAngle(90);
                    absoluteDepositAngle = glyphDepositAngle.add(columnOffset.negative());
                    break;

                case BLUE:
                    glyphDepositAngle = new DegreeAngle(270);
                    absoluteDepositAngle = glyphDepositAngle.add(columnOffset);
                    break;
            }

            Vector vectorOffsetFromColumnAngleOffset = new PolarVector(5, glyphDepositAngle.add(columnOffset));

            // Drive to some offset in front of the cryptobox deposit location.
            robot.ptews.setDesiredPose(new Pose(absoluteDepositLocation.add(vectorOffsetFromColumnAngleOffset), absoluteDepositAngle));
            matchPTEWSDesiredPose(
                    5,
                    new DegreeAngle(3),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 6),
                    null);

            // Flip out
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            // Register the new glyphs
            cryptobox.dump(col, numGlyphs);

            // Push it in
            turnToHeading(glyphDepositAngle, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 2));
            driveTime(
                    new CartesianVector(columnOffset.degrees() < 180 ? .3 : -.3, .4),
                    0,
                    new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 600));
            driveTime(
                    new CartesianVector(0, -.4),
                    0,
                    new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 600));

            // Drive away
            Vector escapeMovement = null;
            double turnPowerEscape = 0.0;
            switch (col)
            {
                case LEFT:
                    escapeMovement = new CartesianVector(.2, .4);
                    turnPowerEscape = 0.0;
                    break;

                case CENTER:
                    escapeMovement = new CartesianVector(0, .5);
                    turnPowerEscape = 0.0;
                    break;

                case RIGHT:
                    escapeMovement = new CartesianVector(-.2, .4);
                    turnPowerEscape = 0.0;
                    break;
            }
            driveTime(escapeMovement, turnPowerEscape, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 600));

            // Flip back
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();
        }
        else
        {
            Pose p = new Pose(
                    (getAlliance() == Alliance.RED ? new CartesianVector(120, 96) : new CartesianVector(24, 96))
                            .add(new CartesianVector(0, 30)), // heading offset from balance board position.

                    new DegreeAngle(getAlliance() == Alliance.BLUE ? 90 : -90));

            turnToHeading(
                    p.heading,
                    10,
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 4));

            int allianceXMultiplier = getAlliance() == Alliance.BLUE ? 1 : -1;
            switch (col)
            {
                case LEFT:
                    p = new Pose(p.position.add(new CartesianVector(allianceXMultiplier * 4, -10)), p.heading);
                    break;
                case CENTER:
                    p = new Pose(p.position.add(new CartesianVector(allianceXMultiplier * 12, -10)), p.heading);
                    break;
                case RIGHT:
                    p = new Pose(p.position.add(new CartesianVector(allianceXMultiplier * 20, -10)), p.heading);
                    break;
            }

            robot.ptews.setDesiredPose(p);
            matchPTEWSDesiredPose(
                    2,
                    new DegreeAngle(5),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 5),
                    null);

            Angle depositOffset = getAlliance() == Alliance.BLUE ?
                    new DegreeAngle(180 - AutonomousSettings.depositAngleOffset) :
                    new DegreeAngle(180 + AutonomousSettings.depositAngleOffset);

            turnToHeading((depositOffset), 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3));

            // Flip out
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            turnToHeading(new DegreeAngle(180), 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3));

            driveTime(new CartesianVector(getAlliance() == Alliance.BLUE ? .3 : -.3, 0), 0, new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
            driveTime(new CartesianVector(0, -.2), 0, new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
            driveTime(new CartesianVector(0, .2), 0, new TimeMeasure(TimeMeasure.Units.SECONDS, 1.5));
        }
    }
}
