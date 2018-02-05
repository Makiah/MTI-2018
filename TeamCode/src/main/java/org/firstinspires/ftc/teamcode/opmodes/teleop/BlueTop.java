package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="Blue Top Teleop", group= Constants.FINAL_BOT_OPMODES)
public class BlueTop extends Teleop
{
    @Override
    public Alliance getAlliance()
    {
        return Alliance.BLUE;
    }

    @Override
    public BalancePlate getBalancePlate()
    {
        return BalancePlate.TOP;
    }
}
