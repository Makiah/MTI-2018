package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankutanku.EnhancedOpMode;
import hankutanku.input.HTButton;
import hankutanku.input.HTGamepad;

@TeleOp(name="Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class Teleop extends EnhancedOpMode
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware);

        while (!isStarted())
            flow.yield();

        while (true)
        {
            HTGamepad.CONTROLLER1.update();
            // HTGamepad.CONTROLLER2.update(); (not currently in use)

            robot.drivetrain.setDriveVector(HTGamepad.CONTROLLER1.leftJoystick());
            robot.drivetrain.setTurnSpeed(HTGamepad.CONTROLLER1.rightJoystick().x());

            if (HTGamepad.CONTROLLER1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.attemptFlipperStateIncrement();
            robot.flipper.updateFlipperState();

            if (gamepad1.right_bumper)
                robot.lift.setPower(1);
            else if (gamepad1.right_bumper)
                robot.lift.setPower(-1);
            else
                robot.lift.setPower(0);

            if (robot.flipper.canIntakeGlyphs())
                robot.harvester.run(gamepad1.left_trigger - gamepad1.right_trigger);
            else
                robot.harvester.run(0);

            flow.yield();
        }
    }
}
