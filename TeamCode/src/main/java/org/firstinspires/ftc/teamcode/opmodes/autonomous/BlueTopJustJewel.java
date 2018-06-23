package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Top - Just Jewel", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class BlueTopJustJewel extends Autonomous
{
    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public BalancePlate getBalancePlate() {
        return BalancePlate.TOP;
    }

    @Override
    protected boolean dontRunAuto() {
        return true;
    }
}
