package org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;

import hankutanku.EnhancedOpMode;
import hankutanku.hardware.SmarterRangeSensor;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.opencv.OpenCVJNIHooks;
import hankutanku.vision.opencv.VisionOpMode;

/**
 * Tracks and guesses the approximate distances from this phone to each individual cryptobox
 * column through a bit of math.
 */
@Autonomous(name="Cryptobox Viewer", group= OpModeDisplayGroups.VISION_TESTING)
public class CryptoboxTracker extends EnhancedOpMode implements VisionOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        // See if we can turn on the lights, but not required.
        try
        {
            DcMotor lights = hardware.initialize(DcMotor.class, "Lights");
            provideLights(lights);
            setLightPowerTo(1);
        }
        catch (Exception e) {}

        // See if we can turn on the range sensor, but not required.
        try
        {
            SmarterRangeSensor rangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Back Range Sensor"), 0x2c);
            provideRangeSensor(rangeSensor);
        }
        catch (Exception e) {}

        // Enable logging.
        setLoggingEnabledTo(true);

        while (true)
            flow.yield();
    }

    @Override
    public Size idealViewResolution()
    {
        return new Size(800, 500);
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation()
    {
        return OpenCVCam.CameraPosition.FRONT;
    }

    @Override
    public boolean enableCameraFlash()
    {
        return false;
    }

    /**
     * Distance forward from the cryptobox can be calculated based on two things: the average width
     * of a column of the box, or the width of the placement location.  If we only see one column,
     * the latter is pointless, and if we only see one placement location, the former is pointless.
     */
    private static final double
            CRYPTO_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST = .14, // The average portions of the screen that the cryptobox cols must take up to be seen as right in front.
            CRYPTO_COL_PROPORTION_WIDTH_FOR_MAX_FORWARD_DIST = .06, // The average portions of the screen for max dist (at edge of visible detection is possible).
            PLACEMENT_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST = .488, // The average portion placement regions must take up for max closest.
            PLACEMENT_COL_PROPORTION_WIDTH_FOR_MAX_FORWARD_DIST = .302; // The average portion placement regions to be taken up for max farthest.

    // The front camera is not positioned dead center.
    private static final double FRONT_CAMERA_VIEW_OFFSET = -.07; // the proportion of the screen to shift for.

    // For logging
    private ProcessConsole trackingConsole;
    public void setLoggingEnabledTo(boolean enabled)
    {
        if (enabled)
        {
            if (trackingConsole == null)
            {
                trackingConsole = LoggingBase.instance.newProcessConsole("Tracking Console");
            }
        }
        else
        {
            if (trackingConsole != null)
            {
                trackingConsole.destroy();
                trackingConsole = null;
            }
        }
    }
    private void updateLoggingConsole()
    {
        if (trackingConsole != null)
            trackingConsole.write(
                    trackingMode == ColumnTrackingMode.COMPLEX ?
                            "Distances are " + placementDistances[0] + ", " + placementDistances[1] + " and " + placementDistances[2] :
                            "Closest is " + closestPlacementLocationOffset,
                    "Estimated forward by col: " + forwardDistanceBasedOnColumns + ", by placement " + forwardDistanceBasedOnPlacementLocations + ", by range " + forwardDistanceBasedOnRange + " result is " + estimatedForwardDistance);
    }

    // Lighting helps a lot, not always on robot tho
    private DcMotor lights = null;
    public void provideLights(DcMotor lights)
    {
        this.lights = lights;
    }
    private void setLightPowerTo(double power)
    {
        if (lights != null)
            lights.setPower(power);
    }

    // Lighting helps a lot, not always on robot tho
    private SmarterRangeSensor rangeSensor = null;
    public void provideRangeSensor(SmarterRangeSensor rangeSensor)
    {
        this.rangeSensor = rangeSensor;
    }
    private double getRangeSensorDist()
    {
        if (rangeSensor != null)
            return rangeSensor.getForwardDist();

        return 255;
    }

    // The alliance for which we'll be doing vision.
    private CompetitionProgram.Alliance alliance = CompetitionProgram.Alliance.BLUE;
    public void setAlliance(CompetitionProgram.Alliance alliance)
    {
        this.alliance = alliance;
    }

    // region Color Filtering
    private enum RedFilteringMode { YCrCb, CMYK }
    private final RedFilteringMode redFilteringMode = RedFilteringMode.YCrCb;
    private final boolean equalizeRedHist = false;

    private enum BlueFilteringMode { YCrCb, CMYK }
    private final BlueFilteringMode blueFilteringMode = BlueFilteringMode.YCrCb;
    private final boolean equalizeBlueHist = false;

    private enum WhiteFilteringMode { Lab, YCrCb }
    private WhiteFilteringMode whiteFilteringMode = WhiteFilteringMode.Lab;
    private final boolean equalizeWhiteHist = false;

    // Pre-chosen for running.
    private boolean needCMYKMat = redFilteringMode == RedFilteringMode.CMYK || blueFilteringMode == BlueFilteringMode.CMYK;
    private boolean needYCrCbMat = redFilteringMode == RedFilteringMode.YCrCb || blueFilteringMode == BlueFilteringMode.YCrCb || whiteFilteringMode == WhiteFilteringMode.YCrCb;
    private boolean needLabMat = whiteFilteringMode == WhiteFilteringMode.Lab;

    /**
     * Converts image to the YCrCb color space and filters by channel to determine red/blue
     * and white filters.
     */
    private void updateColorMasks(Mat main)
    {
        if (main.channels() == 4)
            Imgproc.cvtColor(main, main, Imgproc.COLOR_RGBA2RGB);

        // region Define Channels

        LinkedList<Mat> cmykChannels = null, ycrcbChannels = null, labChannels = null;

        // Get a CMYK mask if any of them need it.
        if (needCMYKMat)
        {
            cmykChannels = new LinkedList<>();
            Mat cmykConvert = main.clone();
            OpenCVJNIHooks.cmykConvert(cmykConvert);
            Core.split(cmykConvert, cmykChannels);
            cmykConvert.release();
        }
        if (needLabMat)
        {
            labChannels = new LinkedList<>();
            Mat labConvert = main.clone();
            Imgproc.cvtColor(labConvert, labConvert, Imgproc.COLOR_RGB2Lab);
            Core.split(labConvert, labChannels);
            labConvert.release();
        }
        if (needYCrCbMat)
        {
            ycrcbChannels = new LinkedList<>();
            Mat ycrcbConvert = main.clone();
            Imgproc.cvtColor(ycrcbConvert, ycrcbConvert, Imgproc.COLOR_RGB2YCrCb);
            Core.split(ycrcbConvert, ycrcbChannels);
            ycrcbConvert.release();
        }

        //endregion

        // region Red Filter

        if (alliance == CompetitionProgram.Alliance.RED)
        {
            Mat redChannel;
            switch (redFilteringMode)
            {
                case CMYK:
                    redChannel = cmykChannels.get(1);
                    if (equalizeRedHist)
                        Imgproc.equalizeHist(redChannel, redChannel);
                    Imgproc.threshold(redChannel, this.primaryMask, 150, 255, Imgproc.THRESH_BINARY);
                    break;

                case YCrCb:
                    redChannel = ycrcbChannels.get(1);
                    if (equalizeRedHist)
                        Imgproc.equalizeHist(redChannel, redChannel);
                    Imgproc.threshold(redChannel, this.primaryMask, 150, 255, Imgproc.THRESH_BINARY);
                    break;
            }
        }

        //endregion

        // region Blue Filter

        else if (alliance == CompetitionProgram.Alliance.BLUE)
        {
            Mat blueChannel;
            switch (redFilteringMode)
            {
                case CMYK:
                    blueChannel = cmykChannels.get(0);
                    if (equalizeBlueHist)
                        Imgproc.equalizeHist(blueChannel, blueChannel);
                    Imgproc.threshold(blueChannel, this.primaryMask, 160 - (1 - estimatedForwardDistance) * 30, 255, Imgproc.THRESH_BINARY);
                    break;

                case YCrCb:
                    blueChannel = ycrcbChannels.get(2);
                    if (equalizeBlueHist)
                        Imgproc.equalizeHist(blueChannel, blueChannel);
                    // more selective filter as we get closer.
                    Imgproc.threshold(blueChannel, this.primaryMask, 135 + (1 - estimatedForwardDistance) * 30, 255, Imgproc.THRESH_BINARY);
                    break;
            }
        }

        //endregion

        // region White Filter

        Mat whiteChannel;
        switch (whiteFilteringMode)
        {
            case Lab:
                whiteChannel = labChannels.get(0);
                if (equalizeWhiteHist)
                    Imgproc.equalizeHist(whiteChannel, whiteChannel);

                Imgproc.threshold(whiteChannel, whiteMask, 150 + (1 - estimatedForwardDistance) * 30, 255, Imgproc.THRESH_BINARY);
                break;

            case YCrCb:
                whiteChannel = ycrcbChannels.get(0);
                if (equalizeWhiteHist)
                    Imgproc.equalizeHist(whiteChannel, whiteChannel);

                Imgproc.threshold(whiteChannel, whiteMask, 150 + (1 - estimatedForwardDistance) * 30, 255, Imgproc.THRESH_BINARY);
                break;
        }

        if (!(alliance == CompetitionProgram.Alliance.BLUE && blueFilteringMode == BlueFilteringMode.CMYK))
        {
            // Eliminate primary mask from white.
            Mat both = Mat.zeros(primaryMask.size(), Imgproc.THRESH_BINARY);
            Core.bitwise_and(primaryMask, whiteMask, both);// If it's part of blue, eliminate it from white.
            Core.bitwise_not(both, both);
            Core.bitwise_and(whiteMask, both, whiteMask);
            both.release();
        }
        else
        {
            // special case: Eliminate white mask from primary.
            Mat both = Mat.zeros(primaryMask.size(), Imgproc.THRESH_BINARY);
            Core.bitwise_and(primaryMask, whiteMask, both);// If it's part of blue, eliminate it from white.
            Core.bitwise_not(both, both);
            Core.bitwise_and(primaryMask, both, primaryMask);
            both.release();
        }

        //endregion
    }

    // endregion

    // Whether or not we try to track and remember column locations.
    public enum ColumnTrackingMode {
        COMPLEX, // Remembers old positions and such.
        SIMPLE   // Just returns the closest offset.
    }
    private ColumnTrackingMode trackingMode = ColumnTrackingMode.SIMPLE;
    public void setTrackingMode(ColumnTrackingMode mode)
    {
        this.trackingMode = mode;
    }

    // Each placement distance for when @trackingMode = ColumnTrackingMode.COMPLEX
    public final int[] placementDistances = new int[3];
    private int lastNumColumnsDetected = 0;

    // Closest placement distance for when @trackingMode = ColumnTrackingMode.SIMPLE
    public double closestPlacementLocationOffset = 1;

    // Classes might want to know this in case zero are detected.
    public boolean detectedNoColumns = true;

    // region Forward Distance Calculation
    // Forward dist from crypto, based on width of detected columns.
    private enum ForwardDistanceDetectionMode { PIXEL_BASED, RANGE_SENSOR, BOTH }
    private ForwardDistanceDetectionMode forwardDistanceDetectionMode = ForwardDistanceDetectionMode.RANGE_SENSOR;
    public double estimatedForwardDistance = 1;
    private double forwardDistanceBasedOnColumns, forwardDistanceBasedOnPlacementLocations, forwardDistanceBasedOnRange;

    /**
     * Used to determine how strict we should be while filtering column area regions.  .
     */
    private void recalculateEstimatedForwardDistance(ArrayList<CryptoColumnPixelLocation> columns)
    {
        if (forwardDistanceDetectionMode != ForwardDistanceDetectionMode.RANGE_SENSOR)
        {
            // region STEP 1: Calculate based on visible cryptobox column pixel widths.

            forwardDistanceBasedOnColumns = 0;

            // Determine average detected cryptobox width
            double avgColWidth = 0;
            int totalValid = 0;
            for (CryptoColumnPixelLocation location : columns) {
                // Make sure this isn't on the edges (not a complete column otherwise).
                if (location.origin < .005 * analysisRegion.height || location.origin + location.width > .995 * analysisRegion.height)
                    continue;

                avgColWidth += location.width;
                totalValid++;
            }
            if (totalValid > 0) {
                // Get average.
                avgColWidth /= totalValid;

                // Get relative to total width.
                avgColWidth /= analysisRegion.height;

                // If avg width = min width portion, then this = max forward dist, if at max then this = 0
                forwardDistanceBasedOnColumns = (avgColWidth - CRYPTO_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST) / (CRYPTO_COL_PROPORTION_WIDTH_FOR_MAX_FORWARD_DIST - CRYPTO_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST);
            }

            // endregion

            // region STEP 2: Calculate based on placement location widths.

            forwardDistanceBasedOnPlacementLocations = 0;

            if (columns.size() >= 2) {
                // Get avg placement width.
                double avgPlacementWidth = 0;

                for (int i = 0; i < columns.size() - 1; i++) {
                    avgPlacementWidth += columns.get(i + 1).origin - (columns.get(i).origin + columns.get(i).width);
                }

                avgPlacementWidth /= columns.size() - 1;

                avgPlacementWidth /= analysisRegion.height;

                forwardDistanceBasedOnPlacementLocations = (avgPlacementWidth - PLACEMENT_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST) / (PLACEMENT_COL_PROPORTION_WIDTH_FOR_MAX_FORWARD_DIST - PLACEMENT_COL_PROPORTION_WIDTH_FOR_MIN_FORWARD_DIST);
            }

            // endregion

            // Now use the results to figure out the distance based on each one.
            if (Math.abs(forwardDistanceBasedOnColumns) < .001)
                estimatedForwardDistance = forwardDistanceBasedOnPlacementLocations;
            else if (Math.abs(forwardDistanceBasedOnPlacementLocations) < .001)
                estimatedForwardDistance = forwardDistanceBasedOnColumns;
            else
                // Average if both were calculated.
                estimatedForwardDistance = (forwardDistanceBasedOnPlacementLocations + forwardDistanceBasedOnColumns) / 2.0;
        }

        if (forwardDistanceDetectionMode != ForwardDistanceDetectionMode.PIXEL_BASED)
        {
            forwardDistanceBasedOnRange = (getRangeSensorDist() - 16) / 18.0;
        }

        // Now calculate the final one.
        switch (forwardDistanceDetectionMode)
        {
            case BOTH:
                estimatedForwardDistance = (((forwardDistanceBasedOnColumns + forwardDistanceBasedOnPlacementLocations) / 2.0) + forwardDistanceBasedOnRange) / 2.0;
                break;

            case PIXEL_BASED:
                estimatedForwardDistance = (forwardDistanceBasedOnColumns + forwardDistanceBasedOnPlacementLocations) / 2.0;
                break;

            case RANGE_SENSOR:
                estimatedForwardDistance = forwardDistanceBasedOnRange;
                break;
        }
    }

    //endregion

    /**
     * Stores the start of the crypto column and the width of the column.
     */
    class CryptoColumnPixelLocation
    {
        public final int origin; // can't be changed
        public int width; // can be modified

        public CryptoColumnPixelLocation(int origin, int width)
        {
            this.origin = origin;
            this.width = width;
        }

        public double midpoint()
        {
            return this.origin + this.width / 2.0;
        }
    }

    // Mat sizes which constitute analysis vs. the size of the frame we were originally passed.
    private Size originalResolution;
    private Size analysisResolution;
    private Rect analysisRegion;

    // Pre-initialized mats.
    private Mat primaryMask, whiteMask;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);
    }

    @Override
    public void onCameraViewStopped()
    {}


    /**
     * Takes a simplified column of crypto colors and decides whether it belongs to a cryptobox.
     */
    private boolean[] quickFilterColumns(Mat mat)
    {
        // Get height of frame (all measurements relative to this).
        int height = mat.cols();

        boolean[] quickFilter = new boolean[mat.rows()];

        for (int rowOfMat = 0; rowOfMat < mat.rows(); rowOfMat++)
        {
            // Get the num of mask pixels in each region.
            int primaryPixels = Core.countNonZero(primaryMask.row(rowOfMat)),
                    whitePixels = Core.countNonZero(whiteMask.row(rowOfMat));

            // Don't analyze if this obviously isn't a column.
            quickFilter[rowOfMat] = primaryPixels > (.6) * height && primaryPixels < .9 * height &&
                    whitePixels > (.04) * height && whitePixels < .6 * height &&
                    (whitePixels + primaryPixels) > (.85) * height;
        }

        return quickFilter;
    }

    /**
     * Decides which columns don't belong from the chosen columns.
     */
    private void deepFilterColumns(Mat mat, boolean[] rows)
    {
        // Use faster C++ method.
        OpenCVJNIHooks.deepCryptoboxAnalysis(mat, primaryMask, whiteMask, estimatedForwardDistance, rows);
    }

    /**
     * Simplifies the one layer boolean array into locations.
     */
    private ArrayList<CryptoColumnPixelLocation> discoverLocationsFrom(boolean[] cryptoColumns)
    {
        // Find distinct regions in binary array
        ArrayList<CryptoColumnPixelLocation> columns = new ArrayList<>();
        int currLength = 0;
        for (int i = 0; i < cryptoColumns.length; i++)
        {
            if (!cryptoColumns[i]) // when it's false, restart the length count but register this series of trues.
            {
                if (currLength > 0)
                    columns.add(new CryptoColumnPixelLocation(i - currLength, currLength));

                currLength = 0;
            }
            else
                currLength++;
        }
        // Otherwise the last column won't be counted.
        if (currLength > 0)
            columns.add(new CryptoColumnPixelLocation(cryptoColumns.length - currLength - 1, currLength));

        // Merge close columns (small column blips)
        final int MERGE_THRESHOLD = (int)(analysisRegion.height / 20.0);
        for (int locIndex = 0; locIndex < columns.size() - 1; locIndex++)
        {
            int locOffset = columns.get(locIndex + 1).origin - (columns.get(locIndex).origin + columns.get(locIndex).width);

            if (locOffset < MERGE_THRESHOLD) // if the end of the first is close enough to the start of the second
            {
                columns.get(locIndex).width += locOffset + columns.get(locIndex + 1).width; // increase size of first col
                columns.remove(locIndex + 1); // remove next col
                locIndex--; // have to reduce count because just considered next col already
            }
        }

        // Remove tiny columns (we're talkin like 1 pixel long)
        final int REMOVE_THRESHOLD = (int)(analysisRegion.height / 50.0);
        for (int i = 0; i < columns.size(); i++)
        {
            if (columns.get(i).width < REMOVE_THRESHOLD)
            {
                columns.remove(i);
                i--;
            }
        }

        return columns;
    }

    /**
     * Complicated method to find equidistant currentTrackers (likely to therefore represent
     * cryptobox).
     */
    private void filterEquidistantColumnsFrom(ArrayList<CryptoColumnPixelLocation> locations)
    {
        // Additional filtering if we detect more than 5 columns.
        final double CLOSE_THRESHOLD = .05 * analysisRegion.width;

        // Find the 4 equidistant columns which represent a cryptobox.
        for (int first = 0; first < locations.size() - 3; first++)
        {
            for (int second = first + 1; second < locations.size() - 2; second++)
            {
                for (int third = second + 1; third < locations.size() - 1; third++)
                {
                    if (!(Math.abs((second - first) - (third - second)) < CLOSE_THRESHOLD))
                        continue;

                    for (int fourth = third + 1; fourth < locations.size(); fourth++)
                    {
                        if (Math.abs((third - second) - (fourth - third)) < CLOSE_THRESHOLD)
                        {
                            for (int i = 0; i < locations.size(); i++)
                            {
                                if (!(i == first || i == second || i == third || i == fourth))
                                {
                                    locations.remove(i);
                                    i--;
                                }
                            }

                            return;
                        }
                    }
                }
            }
        }

        // We haven't found the columns apparently so we'll just remove them from the right end.
        while (locations.size() > 4)
            locations.remove(locations.size() - 1);
    }

    /**
     * Used for finding the closest magnitude of placement distance (from [-30, 40, MAX] returns -30)
     */
    private int getSmallestMagnitudeFrom(int[] array)
    {
        // Find closest distance for tracking.
        int closestIndex = 0;
        for (int i = 1; i < array.length; i++)
            if (Math.abs(placementDistances[i]) < Math.abs(placementDistances[closestIndex]))
                closestIndex = i;

        return closestIndex;
    }

    // region Intelligent Tracking Methods
    /**
     * Sets the placement location if it's within the bounds of placement.
     */
    private void setPlacementLocationIfPossible(int locationIndex, int value)
    {
        if (locationIndex >= 0 && locationIndex < placementDistances.length)
            placementDistances[locationIndex] = value;
    }
    /**
     * Used for intelligent column tracking, just checks whether a placement is the max value currently.
     */
    private boolean isValidLocation(int location)
    {
        return Math.abs(location) != Integer.MAX_VALUE;
    }
    /**
     * Tries to determine placement locations based on the detected columns.
     */
    private void updatePlacementsBasedOn(ArrayList<CryptoColumnPixelLocation> columns)
    {
        // Use the fact that we've recorded closestGlyphPlacementSpace for previous trials if less than 4 detected.
        int centerScreen = (int)(primaryMask.rows() / 2.0);
        switch (columns.size()) // <= 4
        {
            /**
             * There's no real difficulty here, just set each placement distance based on the
             * detected columns.
             */
            case 4:
                for (int i = 0; i < 3; i++)
                    placementDistances[i] = (int)((columns.get(i).midpoint() + columns.get(i + 1).midpoint()) / 2.0 - centerScreen);

                break;

            /**
             * This is a bit harder.
             *
             * If we're transitioning from 4 detected columns to 3, we
             * decide whether the first or last placement location was further away last time,
             * and set that to the max value.  It's less hard because there are only 2 locations
             * it might not be seeing.
             *
             * If we're transitioning from 2 columns, we have to first determine which placement
             * location was the visible one, and then determine which column just reappeared based
             * on that column's offset.
             */
            case 3:
                // We'd seen every column prior.
                if (lastNumColumnsDetected == 4)
                {
                    if (Math.abs(placementDistances[0]) > Math.abs(placementDistances[2]))
                        placementDistances[0] = -Integer.MAX_VALUE;
                    else
                        placementDistances[2] = Integer.MAX_VALUE;
                }
                // We'd only seen one placement box prior.
                else if (lastNumColumnsDetected == 2)
                {
                    // We were just looking at the first placement location, so set middle to +40
                    if (isValidLocation(placementDistances[0]))
                    {
                        placementDistances[1] = 40;
                    }
                    // We were just looking at the third placement location, so set middle to -40
                    else if (isValidLocation(placementDistances[2]))
                    {
                        placementDistances[1] = -40;
                    }
                    // We were just looking at the middle one, so we'll enable first or last depending on the placement dist to te center.
                    else if (isValidLocation(placementDistances[1]))
                    {
                        if (placementDistances[1] < 0)
                            placementDistances[2] = 40;
                        else if (placementDistances[1] > 0)
                            placementDistances[2] = -40;
                    }
                }

                // If we can't see the first column
                if (!isValidLocation(placementDistances[0]))
                {
                    placementDistances[1] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[2] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }
                // If we can't see the last column.
                else if (!isValidLocation(placementDistances[2]))
                {
                    placementDistances[0] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[1] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }

                break;

            /**
             * For 3 -> 2, we have to figure out the closest placement box, and the resulting locations of the others.
             */
            case 2:
                if (lastNumColumnsDetected == 3 || lastNumColumnsDetected == 1)
                {
                    int closestIndex = getSmallestMagnitudeFrom(placementDistances);

                    // Set all except closest index to max.
                    for (int i = 0; i < 3; i++)
                        if (i != closestIndex)
                            placementDistances[i] = (int)(Math.signum(placementDistances[i])) * Integer.MAX_VALUE;
                }

                // Set the one that isn't the max value to the observed value.
                for (int i = 0; i < 3; i++)
                {
                    if (isValidLocation(placementDistances[i]))
                    {
                        placementDistances[i] = (int) ((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                        break;
                    }
                }

                break;

            /**
             * For this one, we just track how the position of one is changing and set the positions
             * arbitrarily high or low but with the correct signs.
             */
            case 1:
                // Decide which two placement locations we'll be deciding between.
                if (lastNumColumnsDetected == 2)
                {
                    int previouslyVisiblePlacementLocation = getSmallestMagnitudeFrom(placementDistances);

                    if (placementDistances[previouslyVisiblePlacementLocation] < 0)
                    {
                        if (previouslyVisiblePlacementLocation < 2)
                            placementDistances[previouslyVisiblePlacementLocation + 1] = 40;
                    }
                    else if (placementDistances[previouslyVisiblePlacementLocation] > 0)
                    {
                        if (previouslyVisiblePlacementLocation > 0)
                            placementDistances[previouslyVisiblePlacementLocation - 1] = -40;
                    }
                    else // This is [hopefully] impossible, it would detect three columns.
                    {}
                }

                // The two indices which are smallest.
                int validIndex1 = -1; // Left one
                int validIndex2 = -1; // Right one.
                if (isValidLocation(placementDistances[0]))
                {
                    if (isValidLocation(placementDistances[1]))
                    {
                        validIndex1 = 0;
                        validIndex2 = 1;
                    }
                    else
                    {
                        // edge placement
                        validIndex1 = -1;
                        validIndex2 = 0;
                    }
                }
                else if (isValidLocation(placementDistances[2]))
                {
                    if (isValidLocation(placementDistances[1]))
                    {
                        validIndex1 = 1;
                        validIndex2 = 2;
                    }
                    else
                    {
                        validIndex1 = 2; // this must be rightmost since other is edge.
                        validIndex2 = 3;
                    }
                }

                int observedLocation = (int)(columns.get(0).midpoint()) - centerScreen;

                if (observedLocation == 0)
                {
                    setPlacementLocationIfPossible(validIndex1, -30);
                    setPlacementLocationIfPossible(validIndex2, 30);
                }
                else if (observedLocation > 0) // on left part of screen so right one is closer.
                {
                    setPlacementLocationIfPossible(validIndex1, -30);
                    setPlacementLocationIfPossible(validIndex2, 40);
                }
                else
                {
                    setPlacementLocationIfPossible(validIndex2, -40);
                    setPlacementLocationIfPossible(validIndex2, 30);
                }

                break;
        }
    }
    // endregion

    /**
     * Finds the closest placement location and sets its offset to a local double.
     */
    private void updateClosestPlacementLocation(ArrayList<CryptoColumnPixelLocation> columns)
    {
        if (columns.size() <= 1)
            return;

        double centerScreen = analysisRegion.height / 2.0;
        double closestLocation = Double.MAX_VALUE;
        for (int i = 0; i < columns.size() - 1; i++)
        {
            double currentOffset = (columns.get(i).midpoint() + columns.get(i + 1).midpoint()) / 2.0 - centerScreen;
            if (Math.abs(currentOffset) < Math.abs(closestLocation))
                closestLocation = currentOffset;
        }

        // With respect to screen size.
        closestLocation /= analysisRegion.height;

        // Account for camera offset.
        closestLocation += FRONT_CAMERA_VIEW_OFFSET;

        // Set closest placement location offset.
        closestPlacementLocationOffset = closestLocation;
    }

    /**
     * Applies the submat to the original mat for display debugging purposes.
     * @param analysisMat analysis submat
     * @param inputFrame  the original mat
     */
    private void applyAnalysisToInput(Mat analysisMat, Mat inputFrame)
    {
        Mat inputSubmatPointer = inputFrame.colRange((int)(analysisRegion.tl().x), (int)(analysisRegion.br().x)).rowRange((int)(analysisRegion.tl().y), (int)(analysisRegion.br().y));
        analysisMat.copyTo(inputSubmatPointer);
        analysisMat.release();
    }

    /**
     * OpenCV method from which everything here is called.
     * @param raw  The Mat which represents the camera view.
     * @return     The Mat which will be displayed on the phone screen.
     */
    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Flip so that it appears normally on the RC.
        Core.flip(raw, raw, 1);

        // Remove pointless alpha channel.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGBA2RGB);

        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Area of interest where the box is illuminated.
        analysisRegion = new Rect(
                new Point(analysisResolution.width * (Range.clip(.325 - (estimatedForwardDistance) * .025, 0, 1)), analysisResolution.height * .05),
                new Point(analysisResolution.width * (Range.clip(.86 - (estimatedForwardDistance) * .1, 0, 1)), analysisResolution.height * .95));

        primaryMask = new Mat(analysisRegion.size(), Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisRegion.size(), Imgproc.THRESH_BINARY);

        // Submat to the region that's consistently lit every match.
        Mat analysisMat = raw.submat(analysisRegion);

        // Remove noise from image.
        Imgproc.blur(analysisMat, analysisMat, new Size(3, 3));

        // Blue/red and white filtering.
        updateColorMasks(analysisMat);

        // Display the results in the display mat.
        analysisMat.setTo(new Scalar(0, 0, 0));
        analysisMat.setTo(new Scalar(255, 255, 255), whiteMask);
        analysisMat.setTo(new Scalar(0, 0, 255), primaryMask);

        // Loop through the columns quickly and eliminate those which obviously aren't cryptobox columns, since we have finite processing power.
        boolean[] cryptoColumns = quickFilterColumns(analysisMat);

        // Further analyze the decided columns to find which ones *probably* don't belong.
        deepFilterColumns(analysisMat, cryptoColumns);

        // Discover the distinct regions of the array from the resulting boolean array.
        ArrayList<CryptoColumnPixelLocation> columns = discoverLocationsFrom(cryptoColumns);

        // Just exit right here if we haven't detected any columns.
        if (columns.size() == 0)
        {
            // Let classes know we can't see the columns.
            detectedNoColumns = true;

            if (forwardDistanceDetectionMode != ForwardDistanceDetectionMode.RANGE_SENSOR)
            {
                // Act as though we've gone further from the columns so our analysis submat changes as well.
                estimatedForwardDistance += .2;

                // Clip to max
                if (estimatedForwardDistance > 1)
                    estimatedForwardDistance = 1;
            }
            else
                // Just uses range sensor to calculate.
                recalculateEstimatedForwardDistance(columns);

            // For visual analysis
            applyAnalysisToInput(analysisMat, raw);

            updateLoggingConsole();

            // Resize the image to the original size.
            Imgproc.resize(raw, raw, originalResolution);
            return raw;
        }

        // Try to filter out false columns if we detected too many.
//        if (columns.size() > 4)
//            filterEquidistantColumnsFrom(columns);

        // Display chosen cols in mat if in debug mode in green (will be overridden if red).
        for (CryptoColumnPixelLocation location : columns)
            Imgproc.rectangle(analysisMat, new Point(0, location.origin), new Point(raw.cols(), location.origin + location.width), new Scalar(0, 255, 0), 3);

        // Update locations based on tracking mode.
        if (trackingMode == ColumnTrackingMode.COMPLEX)
            // Decides what the placement locations should be based on the detected columns.
            updatePlacementsBasedOn(columns);
        else if (trackingMode == ColumnTrackingMode.SIMPLE)
            // Just calculate one.
            updateClosestPlacementLocation(columns);

        // Used for future runs.
        lastNumColumnsDetected = columns.size();

        // We detected columns if we reach this point.
        detectedNoColumns = false;

        // Will be used for future runs to filter the sizes of the blue and white regions.
        recalculateEstimatedForwardDistance(columns);

        // Release masks
        primaryMask.release();
        whiteMask.release();

        // For visual analysis, automatically releases analysisMat.
        applyAnalysisToInput(analysisMat, raw);

        updateLoggingConsole();

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);

        return raw;
    }
}
