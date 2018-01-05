package org.firstinspires.ftc.teamcode.vision;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

/**
 * Tracks and guesses the approximate distances from this phone to each individual cryptobox
 * column through a bit of math.
 */

@Autonomous(name="Cryptobox Tracker", group= Constants.EXPERIMENTATION)
public class CryptoboxTracker extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        ProcessConsole console = LoggingBase.instance.newProcessConsole("Cryptobox Tracker");

        while (true)
        {
            console.write("Front is " + forwardOffset, "Side is " + horizontalOffset);
            flow.yield();
        }
    }

    /**
     * Required for frame analysis.
     */
    private MaskGenerator maskGenerator;

    /**
     * The end doubles that this complex system is trying to calculate.
     */
    public double horizontalOffset = 0, forwardOffset = 0;

    /**
     * Forward dist = dist we can drive forward before hitting the crypto, horizontal dist =
     * dist to the right we can drive before we can reach the center.
     */
    public void provideApproximatePhysicalOffset(double forwardDist, double horizontalOffset)
    {
        this.forwardOffset = forwardDist;
        this.horizontalOffset = horizontalOffset;
    }

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
    }


    // Mat sizes which constitute analysis vs. the size of the frame we were originally passed.
    private Size originalResolution;
    private Size analysisResolution;

    // Pre-initialized mats.
    private Mat blueMask, whiteMask;

    // A single row of detected cryptobox components (just true/false — it's a mask)
    private boolean[] cryptoColumns;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);
        maskGenerator = new MaskGenerator(analysisResolution);

        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped()
    {
        whiteMask.release();
        blueMask.release();
        maskGenerator.releaseMats();
    }

    // Disable this if you don't want to display the current mat state to the user (useful for ensuring everything's working properly but slow)
    private final boolean IN_MAT_DEBUG_MODE = true;

    /**
     * Annoying method to find equidistant locations (likely to therefore represent cryptobox).
     */
    private void getEquidistantColumnsFrom(ArrayList<CryptoColumnPixelLocation> locations)
    {
        // Additional filtering if we detect more than 5 columns.
        final double CLOSE_THRESHOLD = .05 * analysisResolution.width;

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
     * Looks at a group of columns to figure out how far they are from the camera.
     */
    private double getDistanceFromColumns(ArrayList<CryptoColumnPixelLocation> columns)
    {
        // Determine distance from center solely based on each of the 4 columns, relatively easy.
        if (columns.size() == 4)
        {
            // Find average distance from leftmost corner
            double cryptoDist = 0;
            for (CryptoColumnPixelLocation column : columns)
                cryptoDist += column.origin + 0.5 * column.width; // find sum of midpoints
            cryptoDist /= 4.0;

            // Find distance from center of image (where robot currently is located)
            cryptoDist -= 0.5 * analysisResolution.width;
            horizontalOffset = cryptoDist;

            // Figure out vertical offset based on area free to both sides.
            return (columns.get(0).origin + (analysisResolution.width - (columns.get(3).origin + columns.get(3).width))) * (100 / analysisResolution.width) + 59; // approx 30 inches
        }

        // Determine distance from center solely based on 3 columns, relatively hard.
        else if (columns.size() == 3)
        {
            // Ensure that the columns are at least a certain size first.
            double avgSize = 0;
            for (CryptoColumnPixelLocation column : columns)
                avgSize += column.width;
            avgSize /= 3.0;

            if (avgSize > analysisResolution.width * .05)
                // Figure out vertical offset based on area free to both sides.
                return (columns.get(0).origin + (analysisResolution.width - (columns.get(2).origin + columns.get(2).width))) * (10 / analysisResolution.width) + 46;
        }

        // Determine distance from center solely based on 2 columns, even harder.
        else if (columns.size() == 2)
        {
            // Ensure that the columns are at least a certain size first.
            double avgSize = 0;
            for (CryptoColumnPixelLocation column : columns)
                avgSize += column.width;
            avgSize /= 2.0;

            if (avgSize > analysisResolution.width * .1)
                return (columns.get(0).origin + (analysisResolution.width - (columns.get(1).origin + columns.get(1).width))) * (5 / analysisResolution.width) + 30;
        }

        // Determine distance from center based on 1 COLUMN ONLY (GOD MODE difficulty).
        else if (columns.size() == 1)
        {
            if (columns.get(0).width > analysisResolution.width * .3)
                return (columns.get(0).origin + (analysisResolution.width - (columns.get(0).origin + columns.get(0).width))) * (1 / analysisResolution.width);
        }

        return 100;
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        maskGenerator.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Get the blue mask with adaptive hsv.
        maskGenerator.adaptiveHSV(raw, 55, -.1, 135, -.1, 59, 59, 255, blueMask);

        // Get the white mask using just inRange.
        Core.inRange(raw, new Scalar(0, 0, 60), new Scalar(255, 65, 255), whiteMask);

        // Now convert back to normal mode while displaying mats.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);

        // Display the results in the display mat.
        if (IN_MAT_DEBUG_MODE)
        {
            raw.setTo(new Scalar(0, 0, 0));
            raw.setTo(new Scalar(255, 255, 255), whiteMask);
            raw.setTo(new Scalar(0, 0, 255), blueMask);
        }

        // Loop through the columns and eliminate those which aren't cryptobox columns.
        cryptoColumns = new boolean[raw.cols()]; // Represents a single-row binary mask.
        for (int colIndex = 0; colIndex < raw.cols(); colIndex++)
        {
            // Count blue and white pixels from binary masks obtained prior.
            int bluePixels = Core.countNonZero(blueMask.col(colIndex));
            int whitePixels = Core.countNonZero(whiteMask.col(colIndex));

            // Neutralize column if criteria isn't fit.
            boolean sufficientBluePixels = bluePixels > .4 * raw.rows();
            boolean sufficientWhitePixels = whitePixels > .1 * raw.rows() && whitePixels < .5 * bluePixels;

            if (sufficientBluePixels && sufficientWhitePixels)
                cryptoColumns[colIndex] = true;
        }

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

        // Merge close locations (small column blips)
        final int MERGE_THRESHOLD = (int)(analysisResolution.width / 20.0);
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

        // Just exit right here if we haven't detected any columns.
        if (columns.size() == 0)
        {
            // Resize the image to the original size.
            Imgproc.resize(raw, raw, originalResolution);
            return raw;
        }

        // Display chosen cols in mat if in debug mode in red (might be removed).
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnPixelLocation location : columns)
            {
                for (int colIndex = 0; colIndex < location.width; colIndex++)
                {
                    raw.col(location.origin + colIndex).setTo(new Scalar(255, 0, 0));
                }
            }
        }

        // Try to filter out false columns if we detected too many.
        if (columns.size() > 4)
            getEquidistantColumnsFrom(columns);

        // Display chosen cols in mat if in debug mode in green (will be overridden if red).
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnPixelLocation location : columns)
            {
                for (int colIndex = 0; colIndex < location.width; colIndex++)
                {
                    raw.col(location.origin + colIndex).setTo(new Scalar(0, 255, 0));
                }
            }
        }

        // Determine forward dist
        forwardOffset = getDistanceFromColumns(columns);

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);
        return raw;
    }
}
