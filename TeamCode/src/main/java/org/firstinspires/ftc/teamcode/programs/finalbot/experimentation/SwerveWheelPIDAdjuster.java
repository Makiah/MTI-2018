package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveWheel;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.Core;
import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;
import hankextensions.threading.SimpleTaskPackage;

@TeleOp(name="Swerve Wheel PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class SwerveWheelPIDAdjuster extends Core
{
    private SwerveWheel frontLeft, backLeft, frontRight, backRight;
    private SimpleTaskPackage taskPackage;

    @Override
    protected void INITIALIZE() throws InterruptedException {
        taskPackage = new SimpleTaskPackage("Swerve Wheel Adjustments");


        // All of the SwerveWheels (which align on independent threads)
        frontLeft = new SwerveWheel(
                "Front Left",
                null,
                initHardwareDevice(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.013042, 0, 0.000608, 5.194),
                61.58);

        frontRight = new SwerveWheel(
                "Front Right",
                null,
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.012465, 0, 0.000945, 2.5),
                228.38);

        backLeft = new SwerveWheel(
                "Back Left",
                null,
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.0127, 0, 0.000704, 2.85),
                43.636);

        backRight = new SwerveWheel(
                "Back Right",
                null,
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.01304, 0, 0.000669, 5.678),
                257.24);
    }

    @Override
    protected void START() throws InterruptedException
    {
        figureOutPIDConstantsFor(frontLeft);

        figureOutPIDConstantsFor(frontRight);

        figureOutPIDConstantsFor(backLeft);

        figureOutPIDConstantsFor(backRight);
    }

    private ProcessConsole swerveConsole;

    private void figureOutPIDConstantsFor(SwerveWheel swerveWheel) throws InterruptedException
    {
        taskPackage.add(swerveWheel.swivelTask);
        taskPackage.start();
        Vector2D desiredRotation;

        swerveConsole = log.newProcessConsole(swerveWheel.motorName + " PID");

        while (!gamepad1.start)
        {
            desiredRotation = Vector2D.rectangular(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateBy(-90);

            if (desiredRotation.magnitude > .05)
                swerveWheel.setVectorTarget(desiredRotation);

            for (int i = 0; i < 3; i++)
            {
                if (gamepad1.a)
                    swerveWheel.pidController.pidConstants.kP += .0001;
                else if (gamepad1.y)
                    swerveWheel.pidController.pidConstants.kP -= .0001;

                if (gamepad1.b)
                    swerveWheel.pidController.pidConstants.kI += .0001;
                else if (gamepad1.x)
                    swerveWheel.pidController.pidConstants.kI -= .0001;

                if (gamepad1.dpad_up)
                    swerveWheel.pidController.pidConstants.kD += .00001;
                else if (gamepad1.dpad_down)
                    swerveWheel.pidController.pidConstants.kD -= .00001;

                if (gamepad1.dpad_left)
                    swerveWheel.pidController.pidConstants.errorThreshold += .01;
                else if (gamepad1.dpad_right)
                    swerveWheel.pidController.pidConstants.errorThreshold -= .01;

                swerveConsole.write(
                        "kP is " + swerveWheel.pidController.pidConstants.kP,
                        "kI is " + swerveWheel.pidController.pidConstants.kI,
                        "kD is " + swerveWheel.pidController.pidConstants.kD,
                        "error threshold is " + swerveWheel.pidController.pidConstants.errorThreshold,
                        "x = " + gamepad1.left_stick_x + " and y = " + gamepad1.left_stick_y
                );

                Flow.msPause(30);
            }
        }

        taskPackage.pause();
        taskPackage.remove(swerveWheel.swivelTask);
        swerveWheel.turnMotor.setPosition(0.5);

        Flow.msPause(3000);
    }
}
