package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dude.makiah.androidlib.logging.ProcessConsole;
import hankutanku.EnhancedOpMode;
import hankutanku.input.HTButton;
import hankutanku.input.HTGamepad;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.Vector;

@TeleOp(name="Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class Teleop extends EnhancedOpMode
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, AutoOrTeleop.TELEOP, flow);

        robot.relic.setRotatorPosition(false);

        while (!isStarted())
            flow.yield();

        boolean inRelicMode = false;

        ProcessConsole console = log.newProcessConsole("Teleop");

        while (true)
        {
            HTGamepad.CONTROLLER1.update();
            // HTGamepad.CONTROLLER2.update(); (not currently in use)

            if (gamepad1.dpad_up)
            {
                inRelicMode = true;

                robot.harvester.run(0);
            }
            else if (gamepad1.dpad_down)
            {
                inRelicMode = false;

                robot.relic.setExtensionPower(0);
            }

            if (!inRelicMode)
            {
                // Flipper
                if (HTGamepad.CONTROLLER1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.flipper.attemptFlipperStateIncrement();
                robot.flipper.updateFlipperState();

                // Lift
                if (gamepad1.right_bumper)
                    robot.lift.setPower(1);
                else if (gamepad1.left_bumper)
                    robot.lift.setPower(-1);
                else
                    robot.lift.setPower(0);

                // Relic Arm
                if (gamepad1.dpad_left)
                    robot.relic.setExtensionPower(1);
                else if (gamepad1.dpad_right)
                    robot.relic.setExtensionPower(-1);
                else
                    robot.relic.setExtensionPower(0);


                Vector driveVector = HTGamepad.CONTROLLER1.leftJoystick();
                double turnSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;

                // Two different modes: intake and deposit.  The deposit mode results in the trigger being used to slow down the robot.
                if (robot.flipper.canIntakeGlyphs())
                {
                    robot.harvester.run((gamepad1.right_trigger - gamepad1.left_trigger) * .7);
                    if (HTGamepad.CONTROLLER1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                        robot.harvester.toggleFixingStateDisabled();
                    robot.harvester.update();

                    robot.drivetrain.move(driveVector, turnSpeed);
                }
                else
                {
                    robot.harvester.run(0);

                    double powerReductionFactor = (3 * gamepad1.left_trigger + 1);
                    robot.drivetrain.move(driveVector.divide(powerReductionFactor), turnSpeed / powerReductionFactor);
                }
            }
            else
            {
                Vector driveVector = HTGamepad.CONTROLLER1.leftJoystick();
                double turnSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;

                robot.drivetrain.move(driveVector.divide(2), turnSpeed / 2);

                // x for grab
                if (HTGamepad.CONTROLLER1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.relic.toggleGrabbingState();

                // y for rotator
                if (HTGamepad.CONTROLLER1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.relic.toggleRotator();

                robot.relic.setExtensionPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }

            console.write("In " + (inRelicMode ? "relic" : "glyph") + " mode",
                    "Harvester fix mode " + !robot.harvester.fixingStateDisabled);

            flow.yield();
        }
    }
}
