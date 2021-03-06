package org.firstinspires.ftc.teamcode.robot.hardware;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTask;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.math.function.LimitedUpdateRateFunction;
import hankutanku.math.pid.PIDController;

public class EncoderMotor extends ScheduledTask
{
    /**
     * For debugging
     */
    public final String motorName;

    /**
     * The motor reference to which this corresponds.
     */
    public final DcMotor motor;

    /**
     * The PID controller which this motor uses to stabilize itself.
     */
    public final LimitedUpdateRateFunction<Double, Double> errorResponder;

    /**
     * The process console which this motor needs to output data.
     */
    private ProcessConsole processConsole = null;
    public void setEnableLogging(boolean enabled)
    {
        boolean currentlyEnabled = processConsole != null;

        if (enabled == currentlyEnabled)
            return;

        if (enabled)
            processConsole = LoggingBase.instance.newProcessConsole(motorName + " Console");
        else
        {
            processConsole.destroy();
            processConsole = null;
        }
    }

    /**
     * The number of encoder ticks it takes this motor to rotate 360 degrees once.
     */
    private final int ENCODER_TICKS_PER_REVOLUTION;

    /**
     * The wheel circumference which this motor drives (public so that SwomniModule
     * can look at this to know how much to correct by)
     */
    public final double WHEEL_CIRCUMFERENCE;

    /**
     * The encoder motor response methods.
     * @param motorName  The motor name which appears on the console.
     * @param motor  The motor itself.
     * @param motorErrorResponse  The motor error response function
     * @param encoderTicksPerWheelRevolution  duh
     * @param wheelDiameterCM  The diameter of the wheel
     * @param zeroPowerBehavior  The zero power behavior for the motor (whether to actively
     *                           try to maintain position)
     */
    public EncoderMotor(
            String motorName,
            DcMotor motor,
            LimitedUpdateRateFunction motorErrorResponse,
            int encoderTicksPerWheelRevolution,
            double wheelDiameterCM,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        this.motorName = motorName;

        this.motor = motor;
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        resetEncoder();

        this.errorResponder = motorErrorResponse;

        // The wheel which the motor drives.
        ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerWheelRevolution;
        WHEEL_CIRCUMFERENCE = wheelDiameterCM * Math.PI;
    }

    /**
     * Every motor has a different number of encoder ticks per revolution and a distinct wheel
     * circumference, so this just takes that into account while calculating distance traversed.
     */
    public double currentDistanceMoved()
    {
        return ((motor.getCurrentPosition() * 1.0) / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
    }

    /**
     * Resets the motor encoder.
     */
    public void resetEncoder()
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // The desired velocity of the motor in meters per second.
    private double desiredVelocity = 0;
    private double lastMotorPosition = 0;
    private long lastAdjustmentTime = 0;
    private double currentPower = 0;
    private double currentVelocity = 0;

    /**
     * Tells this motor the number of revolutions that it should be moving per second.
     * @param velocity the angular velocity of the motor.
     */
    public void setVelocity(double velocity)
    {
        // Some really quick adjustments we can make.
        if (Math.abs(velocity) < .0001)
        {
            motor.setPower(0);
            currentPower = 0;
            desiredVelocity = 0;
            lastAdjustmentTime = System.currentTimeMillis();
        }
        else
        {
            desiredVelocity = velocity;

            // Dirty approximation, but hey, whatever works :P.  PID adjusts better for this than what it would be.
            if (desiredVelocity < 0 && currentPower > 0)
            {
                motor.setPower(-.1);
                currentPower = -.1;
                lastAdjustmentTime = System.currentTimeMillis();
            }
            else if (desiredVelocity > 0 && currentPower < 0)
            {
                motor.setPower(.1);
                currentPower = .1;
                lastAdjustmentTime = System.currentTimeMillis();
            }
        }
    }

    @Override
    protected TimeMeasure onContinueTask() throws InterruptedException
    {
        // Rare
        if (errorResponder instanceof PIDController)
        {
            if (!((PIDController) errorResponder).canUpdate())
                return TimeMeasure.IMMEDIATE;
        }
        else if (System.nanoTime() - lastAdjustmentTime < errorResponder.getUpdateRate().durationIn(TimeMeasure.Units.NANOSECONDS))
            return TimeMeasure.IMMEDIATE;

        if (Math.abs(desiredVelocity) < .00001)
            return errorResponder.getUpdateRate();

        // Calculate PID by finding the number of ticks the motor SHOULD have gone minus the amount it actually went.
        currentVelocity = (((motor.getCurrentPosition() - lastMotorPosition) / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE) / ((System.nanoTime() - lastAdjustmentTime)) * 1e9;
        currentPower += errorResponder.value(desiredVelocity - currentVelocity);
        currentPower = Range.clip(currentPower, -1, 1);
        motor.setPower(currentPower);

        if (processConsole != null)
            processConsole.write(
                    "Current position: " + lastMotorPosition,
                    "Desired velocity: " + desiredVelocity + " cm/s",
                    "Current velocity: " + currentVelocity + " cm/s",
                    "Current power: " + currentPower,
                    errorResponder instanceof PIDController ? ((PIDController)errorResponder).summary() : "");

        lastMotorPosition = motor.getCurrentPosition();
        lastAdjustmentTime = System.nanoTime();

        return errorResponder.getUpdateRate();
    }
}
