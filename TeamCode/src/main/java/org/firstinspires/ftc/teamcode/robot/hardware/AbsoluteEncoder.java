package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;

public class AbsoluteEncoder
{
    public final AnalogInput device;

    public enum Direction {FORWARD, REVERSE}
    private Direction direction = Direction.FORWARD;
    public void setDirection(Direction direction)
    {
        this.direction = direction;
    }

    public AbsoluteEncoder(AnalogInput device) {
        this.device = device;
    }

    public Angle heading()
    {
        return new DegreeAngle((this.direction == Direction.FORWARD ? 1 : -1) * device.getVoltage() / 3.3 * 360);
    }
}
