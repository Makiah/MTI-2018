package hankutanku.logging;

import android.util.Log;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hankutanku.EnhancedOpMode;

/**
 * Uses my lib to visualize task states on the driver station.
 */
public class TelemetryWrapper extends LoggingBase
{
    private final Telemetry mainTelemetry;

    /**
     * Resets the entire console with empty content.
     */
    public TelemetryWrapper(Telemetry mainTelemetry)
    {
        super(EnhancedOpMode.instance);

        // Set instance and static components.
        this.mainTelemetry = mainTelemetry;

        // Start the updater ASAP.
        this.run();
    }

    /**
     * Rebuilds the whole telemetry console (call minimally, allow the task to take care of it.)
     */
    @Override
    protected void refreshOnScreenConsole()
    {
        if (mainTelemetry != null)
        {
            //Add all private console data.
            for (ProcessConsole pConsole : privateProcessConsoles)
            {
                mainTelemetry.addLine ("----- " + pConsole.processName + " -----");

                for (String line : pConsole.processData)
                    mainTelemetry.addLine (line);

                mainTelemetry.addLine ("");
            }

            mainTelemetry.addLine ("----- Sequential Data -----");
            for (String line : sequentialConsoleData)
            {
                mainTelemetry.addLine (line);
            }

            //Refresh the console with this new data.
            mainTelemetry.update ();
        }

        StringBuilder logMessage = new StringBuilder();
        if (mainTelemetry != null)
        {
            //Add all private console data.
            for (ProcessConsole pConsole : privateProcessConsoles)
            {
                logMessage.append("----- ").append(pConsole.processName).append(" -----\n");

                for (String line : pConsole.processData)
                    logMessage.append (line + "\n");

                logMessage.append("\n");
            }

            logMessage.append ("----- Sequential Data -----");
            for (String line : sequentialConsoleData) {
                logMessage.append(line + "\n");
            }
        }

        Log.e("HankuTelem", logMessage.toString());

        //Otherwise it just gets queued in the ArrayList.
    }
}