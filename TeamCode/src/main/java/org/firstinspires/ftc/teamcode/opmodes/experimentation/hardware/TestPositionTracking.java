package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.IncrementalAbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.PoseTrackingEncoderWheelSystem;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;

@Autonomous(name="Test Pose Tracking", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestPositionTracking extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        PoseTrackingEncoderWheelSystem ptews = new PoseTrackingEncoderWheelSystem(
                new AbsoluteEncoder(hardwareMap.analogInput.get("left tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("center tracking wheel")),
                new AbsoluteEncoder(hardwareMap.analogInput.get("right tracking wheel"))
        );

        ptews.provideExternalPoseInformation(new Pose(Pose.PoseType.ABSOLUTE, new CartesianVector(72, 48), new DegreeAngle(0)));

        while (true)
        {
            ptews.update();

            flow.pause(new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 20));
        }
    }
}
