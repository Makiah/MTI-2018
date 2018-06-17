package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;

public class AbsoluteEncoder
{
    public final AnalogInput device;

    public AbsoluteEncoder(AnalogInput device) {
        this.device = device;
    }

    public Angle heading()
    {
        return new DegreeAngle(device.getVoltage() / 3.3 * 360);
    }
}
