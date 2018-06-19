package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.ClampingFlipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Harvester;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;
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
    public final Harvester harvester;
    public final DcMotor relic;
    public final PoseTrackingEncoderWheelSystem ptews;

    public Robot(HardwareInitializer initializer)
    {
        drivetrain = new SpeedyMecanumDrive(
                initializer.initialize(DcMotor.class, "front left"),
                initializer.initialize(DcMotor.class, "rear left"),
                initializer.initialize(DcMotor.class, "rear right"),
                initializer.initialize(DcMotor.class, "front right")
        );

        flipper = new ClampingFlipper(
                initializer.initialize(Servo.class, "servo0"), // left flipper
                initializer.initialize(Servo.class, "servo1"), // right flipper
                initializer.initialize(Servo.class, "servo2"), // lower clamp
                initializer.initialize(Servo.class, "servo3")
        );

        lift = initializer.initialize(DcMotor.class, "lift");

        harvester = new Harvester(
                initializer.initialize(DcMotor.class, "left harvester"),
                initializer.initialize(DcMotor.class, "right harvester")
        );

        relic = initializer.initialize(DcMotor.class, "relic");

        initializer.initialize(Servo.class, "servo4").setPosition(0); // jewel knocker (temporary).

        ptews = new PoseTrackingEncoderWheelSystem(
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "left tracking wheel")),
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "center tracking wheel")),
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "right tracking wheel"))
        );
    }
}
