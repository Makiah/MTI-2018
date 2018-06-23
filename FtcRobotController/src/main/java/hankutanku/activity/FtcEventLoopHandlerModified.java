/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package hankutanku.activity;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("WeakerAccess")
public class FtcEventLoopHandlerModified extends FtcEventLoopHandler {

  public static String lastBatterySend = "";

  public FtcEventLoopHandlerModified(HardwareFactory hardwareFactory, UpdateUI.Callback callback, Context robotControllerContext) {
    super(hardwareFactory, callback, robotControllerContext);
  }

  public BatteryChecker getRobotControllerBatteryChecker() {
    return robotControllerBatteryChecker;
  }

  private String buildRobotBatteryMsg() {

    // Don't do anything if we're really early in the construction cycle
    if (this.hardwareMap==null) return null;

    double minBatteryLevel = Double.POSITIVE_INFINITY;

    // Determine the lowest battery voltage read from all motor controllers.
    //
    // If a voltage sensor becomes disconnected, it has been observed to read as zero.
    // Thus, we must account for that eventuality. While doing so, it's convenient for us
    // to rule out (other) unreasonable voltage levels in order to facilitate later string
    // conversion.
    //
    for (VoltageSensor sensor : this.hardwareMap.voltageSensor) {

      // Read the voltage, keeping track of how long it takes to do so
      long nanoBefore = System.nanoTime();
      double sensorVoltage = sensor.getVoltage();
      long nanoAfter = System.nanoTime();

      if (sensorVoltage >= 1.0 /* an unreasonable value to ever see in practice */) {
        // For valid reads, we add the read-duration to our statistics, in ms.
        robotBatteryStatistics.add((nanoAfter - nanoBefore) / (double) ElapsedTime.MILLIS_IN_NANO);

        // Keep track of the minimum valid value we find
        if (sensorVoltage < minBatteryLevel) {
          minBatteryLevel = sensorVoltage;
        }
      }
    }

    String msg;

    if (minBatteryLevel == Double.POSITIVE_INFINITY) {
      msg = NO_VOLTAGE_SENSOR;

    } else {
      // Convert double voltage into string with *two* decimal places (fast), given the
      // above-maintained fact the voltage is at least 1.0.
      msg = Integer.toString((int)(minBatteryLevel * 100));
      msg = new StringBuilder(msg).insert(msg.length()-2, ".").toString();
    }

    return (msg);
  }

  @Override
  public void sendBatteryInfo() {
    robotControllerBatteryChecker.pollBatteryLevel(this);
    String batteryMessage = buildRobotBatteryMsg();
    if (batteryMessage != null) {
      sendTelemetry(EventLoopManager.ROBOT_BATTERY_LEVEL_KEY, batteryMessage);

      lastBatterySend = batteryMessage;
    }
  }
}
