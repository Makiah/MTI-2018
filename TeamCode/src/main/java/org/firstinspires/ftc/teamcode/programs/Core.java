package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sdkextensions.logging.Log;
import org.firstinspires.ftc.teamcode.sdkextensions.music.Tunes;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

/**
 * NiFTBase is the class from which all user OpModes should inherit.  With advanced error handling, it takes care of the scenarios in which the user requests an early stop, fails to take an error into account, etc.
 */
public abstract class Core extends LinearOpMode
{
    /**
     * Useful for other files which require custom initialization steps or components from this op mode which they cannot otherwise obtain.
     */
    public static LinearOpMode current;
    public static Log log;

    /**
     * runOpMode() is the method called by LinearOpMode to start the program, but is really low-level.  What this method does is split the sequence into a set of steps which every autonomous program should include, while also observing errors and either stopping the code or outputting them based on their severity.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode () throws InterruptedException
    {
        try
        {
            //Classes such as NiFTMusic require this so that they can get the context they require.
            current = this;
            log = new Log(telemetry);
            log.startConsoleUpdater();

            //REQUIRED in child classes.
            HARDWARE();

            //May be used in different programs.
            INITIALIZE();

            //Wait for the start button to be pressed.
            waitForStart ();

            //This is where the child classes mainly differ in their instructions.
            START();
        }
        catch (InterruptedException e) {} //If this is caught, then the user requested program stop.
        catch (Exception e) //If this is caught, it wasn't an InterruptedException and wasn't requested, so the user is notified.
        {
            log.newLine("UH OH!  An error was just thrown!");
            log.newLine(e.getMessage ());
            log.newLine("Will end upon tapping stop...");

            //Wait until stop is requested.
            try
            {
                while (true)
                    Flow.yield ();
            }
            catch (InterruptedException e2) {} //The user has read the message and stops the program.
        }
        finally //Occurs after all possible endings.
        {
            Tunes.silence();
            STOP();
        }
    }

    /**
     * In this method, initialize all required hardware (motors, servos, etc.).
     */
    protected void HARDWARE() throws InterruptedException {};
    /**
     * In this method, do everything that needs to happen AFTER hardware initialization and during Init (like
     * gyro calibration).
     */
    protected void INITIALIZE() throws InterruptedException {}
    /**
     * Code everything that the robot needs to do upon play being tapped in here.
     */
    protected abstract void START() throws InterruptedException;
    /**
     * Any final actions that need to happen.
     */
    protected void STOP() {}
}