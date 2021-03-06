/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Debounce", group = "Concept")
//@Disabled
public class debounce extends OpMode {
  boolean SlowMode = false;
  boolean yButtonIsReleased = true;
  boolean xButtonIsReleased = true;
  int position, pos2, pos3, pos4;
  int inc = 1;
  private ElapsedTime runtime = new ElapsedTime();
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
  }
  @Override
  public void start() {
    runtime.reset();
  }
  @Override
  public void loop() {
    if (gamepad1.y) {
      if (yButtonIsReleased){
        yButtonIsReleased = false;
        SlowMode = !SlowMode;
      }
      } else {
      yButtonIsReleased = true;
    }
    if (gamepad1.x) {
      if (xButtonIsReleased){
        xButtonIsReleased = false;
        position += inc;
      }
    } else {
      xButtonIsReleased = true;
    }
    if (gamepad1.a) pos2 += inc;
    else pos3 += inc;
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("1", "SlowMode: " + SlowMode);
    telemetry.addData("2", "position: " + position);
    telemetry.addData("3", "a pressed " + pos2);
    telemetry.addData("4", "a not " + pos3);
    telemetry.addData("5", "left trig " + gamepad1.left_trigger);
  }
}
