/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * {@link MultiI2CLoop} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Multi Latency loop", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class MultiI2CLoop extends OpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
        private ElapsedTime runtime = new ElapsedTime();
        double lastTime;
        double sumTime;
        double avgTime;
        int cntr;
        double maxTime =0;
        double minTime =100000;
        boolean firstTime = true;
        double IRVal;
        AnalogInput IR;
        IrSeekerSensor irSeeker;
        NormalizedColorSensor colorSensor;
        ColorSensor color1;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void init() {


        irSeeker = hardwareMap.irSeekerSensor.get("sensor_ir");


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color sensor");

        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color1.setI2cAddress((I2cAddr.create7bit(0x20)));
        color1.enableLed(true);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        IR = hardwareMap.analogInput.get("IR");
    }
        // Wait until we're told to go

@Override
        public void start()

        {
            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            runtime.reset();
        }
        // Loop and update the dashboard
        @Override public void loop() {
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double oneTime = runtime.time() - lastTime;
            lastTime = runtime.time();
            if (firstTime) {
                firstTime = false;
            } else if (oneTime < minTime) minTime = oneTime;
            if  (oneTime > maxTime) maxTime = oneTime;
            if (cntr > 999) {
                avgTime = sumTime / 1000;
                sumTime =0;
                cntr = 0;
            }
            else  {
                cntr += 1;
                sumTime += oneTime;
            }

            telemetry.addData("3", "angles.firstAngle " + angles.firstAngle);
            telemetry.addData("4", "each loop time " + oneTime * 1000);
            telemetry.addData("5", "avg loop time " + avgTime * 1000);
            telemetry.addData("6", "minTime " + minTime * 1000);
            telemetry.addData("7", "maxTime " + maxTime * 1000);
            telemetry.addData("10", "iR " + IR.getVoltage());
            if (irSeeker.signalDetected())
            {
                // Display angle and strength
                telemetry.addData("Angle",    irSeeker.getAngle());
                telemetry.addData("Strength", irSeeker.getStrength());
            }
            else
            {
                // Display loss of signal
                telemetry.addData("Seeker", "Signal Lost");
            }
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            telemetry.addData("a", "%.3f", colors.alpha);
            telemetry.addData("8","color1 a " + color1.alpha());
        }
       // color1.enableLed(false);
    }

