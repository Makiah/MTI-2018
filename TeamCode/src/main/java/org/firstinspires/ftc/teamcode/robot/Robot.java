package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.ClampingFlipper;
import org.firstinspires.ftc.teamcode.robot.hardware.SpeedyMecanumDrive;

import hankutanku.hardware.HardwareInitializer;

/**
 * Wrapper class for all robot hardware.
 */
public class Robot
{
    public final SpeedyMecanumDrive drivetrain;
    public final ClampingFlipper flipper;
    public final DcMotor lift;

    public Robot(HardwareInitializer initializer)
    {
        drivetrain = new SpeedyMecanumDrive(
                initializer.initialize(DcMotor.class, "Front Left"),
                initializer.initialize(DcMotor.class, "Back Left"),
                initializer.initialize(DcMotor.class, "Back Right"),
                initializer.initialize(DcMotor.class, "Front Right")
        );

        flipper = new ClampingFlipper(
                initializer.initialize(Servo.class, "servo0"), // left flipper
                initializer.initialize(Servo.class, "servo1"), // right flipper
                initializer.initialize(Servo.class, "servo2"), // lower clamp
                initializer.initialize(Servo.class, "servo3")
        );

        lift = initializer.initialize(DcMotor.class, "lift");
    }
}
