package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.ClampingFlipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Harvester;
import org.firstinspires.ftc.teamcode.robot.hardware.JewelKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.RelicArm;
import org.firstinspires.ftc.teamcode.robot.hardware.SpeedyMecanumDrive;

import dude.makiah.androidlib.threading.Flow;
import hankutanku.EnhancedOpMode;
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
    public final RelicArm relic;
    public final PoseTrackingEncoderWheelSystem ptews;
    public final JewelKnocker jewelKnocker;
    public DistanceSensor glyphSensor = null, glyphSensor2 = null;

    public Robot(HardwareInitializer initializer, EnhancedOpMode.AutoOrTeleop autoOrTeleop, Flow flow) throws InterruptedException
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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        harvester = new Harvester(
                initializer.initialize(DcMotor.class, "left harvester"),
                initializer.initialize(DcMotor.class, "right harvester")
        );

        jewelKnocker = new JewelKnocker(
                initializer.initialize(Servo.class, "servo4"),
                initializer.initialize(Servo.class, "servo5")
        );

        ptews = new PoseTrackingEncoderWheelSystem(
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "left tracking wheel")),
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "center tracking wheel")),
                new AbsoluteEncoder(initializer.initialize(AnalogInput.class, "right tracking wheel"))
        );

        relic = new RelicArm(
                initializer.initialize(DcMotor.class, "relic"),
                initializer.initialize(Servo.class, "servo6"),
                initializer.initialize(Servo.class, "servo7")
        );

        if (autoOrTeleop == EnhancedOpMode.AutoOrTeleop.AUTONOMOUS)
        {
            glyphSensor = initializer.map.get(DistanceSensor.class, "glyph sensor");
            glyphSensor2 = initializer.map.get(DistanceSensor.class, "glyph sensor 2");
        }
    }
}
