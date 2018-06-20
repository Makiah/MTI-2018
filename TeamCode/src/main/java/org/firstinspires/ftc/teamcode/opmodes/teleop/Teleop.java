package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankutanku.EnhancedOpMode;
import hankutanku.input.HTButton;
import hankutanku.input.HTGamepad;
import hankutanku.math.angle.DegreeAngle;

@TeleOp(name="Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class Teleop extends EnhancedOpMode
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, AutoOrTeleop.TELEOP);

        while (!isStarted())
            flow.yield();

        robot.jewelKnocker.getOuttaTheWay();

        while (true)
        {
            HTGamepad.CONTROLLER1.update();
            // HTGamepad.CONTROLLER2.update(); (not currently in use)

            if (HTGamepad.CONTROLLER1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.attemptFlipperStateIncrement();
            robot.flipper.updateFlipperState();

            if (gamepad1.right_bumper)
                robot.lift.setPower(1);
            else if (gamepad1.left_bumper)
                robot.lift.setPower(-1);
            else
                robot.lift.setPower(0);

            if (gamepad1.dpad_left)
                robot.relic.setPower(1);
            else if (gamepad1.dpad_right)
                robot.relic.setPower(-1);
            else
                robot.relic.setPower(0);

            if (robot.flipper.canIntakeGlyphs())
            {
                robot.harvester.run((gamepad1.left_trigger - gamepad1.right_trigger) * .7);
                robot.drivetrain.move(HTGamepad.CONTROLLER1.leftJoystick(), HTGamepad.CONTROLLER1.gamepad.right_stick_x);
            }
            else
            {
                robot.harvester.run(0);

                double powerReductionFactor = (3 * gamepad1.left_trigger + 1);
                robot.drivetrain.move(HTGamepad.CONTROLLER1.leftJoystick().divide(powerReductionFactor), HTGamepad.CONTROLLER1.gamepad.right_stick_x / powerReductionFactor);
            }

            flow.yield();
        }
    }
}
