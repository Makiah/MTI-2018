package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousBase;

@TeleOp(name="Red Bottom Teleop", group= Constants.FINAL_BOT_OPMODES)
public class RedBottom extends AutonomousBase
{
    @Override
    public Alliance getAlliance()
    {
        return Alliance.RED;
    }

    @Override
    public BalancePlate getBalancePlate()
    {
        return BalancePlate.BOTTOM;
    }
}