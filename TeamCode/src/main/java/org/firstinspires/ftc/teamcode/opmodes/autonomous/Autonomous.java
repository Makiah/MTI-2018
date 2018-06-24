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

    protected abstract boolean dontAttemptGlyphs();

    private Pose balancePlatePoseFollowingDrive = null;
    private Pose getBalancePlatePoseFollowingDrive()
    {
        if (balancePlatePoseFollowingDrive == null)
        {
            Vector balancePlateOffset = new CartesianVector(24, 24); // starts at blue bottom
            Angle balancePlateHeading = new DegreeAngle(0);
            if (getBalancePlate() == BalancePlate.TOP)
                balancePlateOffset = balancePlateOffset.add(new CartesianVector(0, 72));
            if (getAlliance() == Alliance.RED) {
                balancePlateOffset = balancePlateOffset.add(new CartesianVector(96, 0));
                balancePlateHeading = new DegreeAngle(180);
            }
            log.lines("Starting position is " + balancePlateOffset.toString(false));

            balancePlatePoseFollowingDrive = new Pose(balancePlateOffset, balancePlateHeading).add(new CartesianVector(0, 28));
        }

        return balancePlatePoseFollowingDrive;
    }

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
        cryptobox = new Cryptobox(getAlliance(), getBalancePlate());

        // Gyro
        AndroidGyro phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();
        phoneGyro.initAntiDrift();

        // Vision
        OpenCVCam openCVCam = new OpenCVCam();
        VuforiaCam vuforiaCam = new VuforiaCam();
        JewelDetector2 jewelDetector = new JewelDetector2();

        // region Initialization Detection of the Crypto Key and Jewel Alignment

        // DON'T specify a default order, if we mess this up we lose points.
        JewelDetector2.JewelOrder jewelOrder = JewelDetector2.JewelOrder.UNKNOWN;
        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;

        // Loop through
        ProcessConsole observedConsole = log.newProcessConsole("Observed Init stuff");
        while (true) // Runs until OpMode is started, then just goes from there.
        {
            // Jewel detection
            openCVCam.start(jewelDetector);

            long startOpenCV = System.currentTimeMillis();
            while (System.currentTimeMillis() - startOpenCV < 2000)
            {
                if (isStarted() && jewelOrder != JewelDetector2.JewelOrder.UNKNOWN)
                    break;

                flow.yield();
            }

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


        /************************/


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

        if (dontAttemptGlyphs())
        {
            flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
            return;
        }


        /************************/


        // Steady drive (manual) off the balance board, using gyro to determine when we're level (aka off the platform).
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


        /************************/


        // Set the positioning system's offset
        robot.ptews.reset();
        robot.ptews.setCurrentPose(getBalancePlatePoseFollowingDrive()); // untracked distance.

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
     * @param fieldCentricDrive  vector to drive
     * @param turnSpeed  turn speed
     * @param driveTime  time to drive
     * @throws InterruptedException
     */
    public void driveTime(Vector fieldCentricDrive, double turnSpeed, TimeMeasure driveTime) throws InterruptedException
    {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < driveTime.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            robot.drivetrain.move(fieldCentricDrive.rotateBy(robot.ptews.getCurrentPose().heading.negative()), turnSpeed);
            robot.ptews.update();
            flow.yield();
        }

        robot.drivetrain.stop();
    }

    /**
     * Drives the robot by some vector and turn speed for some length of time
     * @param fieldCentricDrive  vector to drive
     * @param desiredHeading  desired heading during drive
     * @param driveTime  time to drive
     * @throws InterruptedException
     */
    public void driveTime(Vector fieldCentricDrive, Angle desiredHeading, TimeMeasure driveTime) throws InterruptedException
    {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < driveTime.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            robot.drivetrain.move(fieldCentricDrive.rotateBy(robot.ptews.getCurrentPose().heading.negative()), desiredHeading.quickestDegreeMovementTo(robot.ptews.getCurrentPose().heading) / 20.0);
            robot.ptews.update();
            flow.yield();
        }

        robot.drivetrain.stop();
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
    public void matchPTEWSDesiredPose(double minimumInchesFromTarget, Angle minimumHeadingFromTarget, TimeMeasure timeoutLength, Function<Void, Double> completionBasedFunction) throws InterruptedException
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

            // Current position and heading from tracking encoders
            Pose currentPose = robot.ptews.getCurrentPose();
            if (Double.isNaN(originalTargetOffset))
                originalTargetOffset = currentPose.position.magnitude();

            // Offset from target position
            Vector positionalOffsetFromTarget = robot.ptews.getDesiredPose().position.subtract(currentPose.position);
            double headingDegreeOffsetFromTarget = currentPose.heading.quickestDegreeMovementTo(currentPose.heading);

            // Whether we can cancel the movement.
            if (positionalOffsetFromTarget.magnitude() <= minimumInchesFromTarget && Math.abs(headingDegreeOffsetFromTarget) <= minimumHeadingFromTarget.degrees())
            {
                LoggingBase.instance.lines("Quit movement because position offset is " + positionalOffsetFromTarget.toString(false) + " and heading offset is " + headingDegreeOffsetFromTarget);
                break;
            }

            // Calculate how to move toward the target pose.
            Vector driveVector = positionalOffsetFromTarget.rotateBy(currentPose.heading.negative()).divide(20);
            driveVector = new PolarVector(Range.clip(driveVector.magnitude(), -AutonomousSettings.maxMoveSpeed, AutonomousSettings.maxMoveSpeed), driveVector.angle());
            double turnSpeed = Range.clip(headingDegreeOffsetFromTarget / 20, -AutonomousSettings.maxTurnSpeed, AutonomousSettings.maxTurnSpeed);
            robot.drivetrain.move(driveVector, turnSpeed);

            // Determine whether we should power movement up or down.
            if (previousPositionalOffset != null)
            {
                Vector movedSinceLastLoop = positionalOffsetFromTarget.subtract(previousPositionalOffset);
                movementPowerUpFactor += (.03 - (movedSinceLastLoop.magnitude() / previousPositionalOffset.magnitude())) * .5; // should have moved 10 percent more of the way toward our target destination.
            }
            previousPositionalOffset = positionalOffsetFromTarget;

            // Determine whether we should power turning up or down.
            if (!Double.isNaN(previousHeadingOffset))
            {
                double degreesMovedSinceLastLoop = new DegreeAngle(headingDegreeOffsetFromTarget).quickestDegreeMovementTo(new DegreeAngle(previousHeadingOffset));
                headingPowerUpFactor += (.03 - (Math.abs(degreesMovedSinceLastLoop) / Math.abs(previousHeadingOffset))) * .06; // should have moved 10 percent more of the required heading.
            }
            previousHeadingOffset = headingDegreeOffsetFromTarget;

            // Call any user-defined lambdas.
            if (completionBasedFunction != null)
                completionBasedFunction.value((1.0 - positionalOffsetFromTarget.magnitude()) / originalTargetOffset);

            // Write console data
            console.write(
                    "Target pose is " + robot.ptews.getCurrentPose().toString(),
                    "Positional offset is " + positionalOffsetFromTarget.toString(false),
                    "Heading offset is " + Vector.decimalFormat.format(headingDegreeOffsetFromTarget),
                    "Movement power up is " + movementPowerUpFactor,
                    "Heading power up is " + headingPowerUpFactor);

            // Pause so that we don't update instantaneously.
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
            // Drive to some offset in front of the cryptobox deposit location.
            robot.ptews.setDesiredPose(cryptobox.getDepositPoseFor(col));
            matchPTEWSDesiredPose(
                    5,
                    new DegreeAngle(3),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 6),
                    null);

            // Flip out
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            // Push them in
            driveTime(new PolarVector(.4, cryptobox.unmodifiedDepositAngle), cryptobox.unmodifiedDepositAngle, new TimeMeasure(TimeMeasure.Units.SECONDS, 2));

            // Register the new glyphs
            cryptobox.dump(col, numGlyphs);

            // Drive away
            int allianceMultipler = getAlliance() == Alliance.BLUE ? 1 : -1;
            Vector escapeMovement = null;
            switch (col)
            {
                case LEFT:
                    escapeMovement = new CartesianVector(.4 * allianceMultipler, .2);
                    break;

                case CENTER:
                    escapeMovement = new CartesianVector(.5 * allianceMultipler, 0);
                    break;

                case RIGHT:
                    escapeMovement = new CartesianVector(.4 * allianceMultipler, -.2);
                    break;
            }
            driveTime(escapeMovement, new DegreeAngle(0), new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 600));

            // Flip back
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();
        }
        else
        {
            Angle driveHeading = new DegreeAngle(getAlliance() == Alliance.BLUE ? 90 : -90);

            turnToHeading(
                    driveHeading,
                    10,
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 4));

            robot.ptews.setDesiredPose(new Pose(cryptobox.getDepositPoseFor(col).position, driveHeading));
            matchPTEWSDesiredPose(
                    2,
                    new DegreeAngle(5),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 5),
                    null);

            turnToHeading(cryptobox.getDepositPoseFor(col).heading, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3));

            // Flip out
            robot.flipper.attemptFlipperStateIncrement();
            while (robot.flipper.isMidStateTransition())
                robot.flipper.updateFlipperState();

            turnToHeading(cryptobox.unmodifiedDepositAngle, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3));

            driveTime(new CartesianVector(0, .3), 0, new TimeMeasure(TimeMeasure.Units.SECONDS, 1));
            driveTime(new CartesianVector(0, -.3), 0, new TimeMeasure(TimeMeasure.Units.SECONDS, 1.5));
        }
    }
}
