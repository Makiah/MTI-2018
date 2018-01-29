package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.structs.PIDController;

public class EncoderMotor
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
    public final PIDController pidController;

    /**
     * The process console which this motor needs to output data.
     */
    private final ProcessConsole processConsole;

    /**
     * The number of encoder ticks it takes this motor to rotate 360 degrees once.
     */
    private final int ENCODER_TICKS_PER_REVOLUTION;

    /**
     * The wheel circumference which this motor drives (public so that SwerveModule
     * can look at this to know how much to correct by)
     */
    public final double WHEEL_CIRCUMFERENCE;

    /**
     * For custom PID control.
     */
    public EncoderMotor(String motorName, DcMotor motor, PIDController motorPID, int encoderTicksPerWheelRevolution, double wheelDiameterCM, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        this.motorName = motorName;

        this.motor = motor;
        resetEncoder();

        this.pidController = motorPID;

        // The wheel which the motor drives.
        ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerWheelRevolution;
        WHEEL_CIRCUMFERENCE = wheelDiameterCM * Math.PI;

        processConsole = LoggingBase.instance.newProcessConsole(motorName + " Motor Process Console");

        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Every motor has a different number of encoder ticks per revolution and a distinct wheel
     * circumference, so this just takes that into account while calculating distance traversed.
     */
    public double currentDistanceMoved()
    {
        return ((motor.getCurrentPosition()) / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
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
    private double currentPower = 0, currentVelocity = 0;

    /**
     * Tells this motor the number of revolutions that it should be moving per second.
     * @param velocity the angular velocity of the motor.
     */
    public void setVelocity(double velocity)
    {
        // Some really quick adjustments we can make.
        if (Math.abs(velocity) < .001) {
            motor.setPower(0);
            currentPower = 0;
        } else if (currentPower < 0 && desiredVelocity > 0) {
            motor.setPower(.1);
            currentPower = .1;
        } else if (currentPower > 0 && desiredVelocity < 0) {
            motor.setPower(-.1);
            currentPower = -.1;
        }

        desiredVelocity = velocity;
    }

    /**
     * Controls updating PID for the motor.
     */
    public void updatePID()
    {
        // Rare
        if (System.nanoTime() - lastAdjustmentTime == 0)
            return;

        if (!pidController.canUpdate())
            return;

        // Calculate PID by finding the number of ticks the motor SHOULD have gone minus the amount it actually went.
        currentVelocity = (((motor.getCurrentPosition() - lastMotorPosition) / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE) / ((System.nanoTime() - lastAdjustmentTime)) *  1e9; // big # is for seconds to nanoseconds conversion.
        currentPower += pidController.calculatePIDCorrection(desiredVelocity - currentVelocity);
        motor.setPower(Range.clip(currentPower, -1, 1));

        processConsole.write(
                "Current position: " + lastMotorPosition,
                "Desired velocity: " + desiredVelocity + " cm/s",
                "Current velocity: " + currentVelocity + " cm/s",
                "Current power: " + currentPower,
                "PID constants: " + pidController.kP + ", " + pidController.kD);

        lastMotorPosition = motor.getCurrentPosition();
        lastAdjustmentTime = System.nanoTime();
    }
}
