// Change for sprockets
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * AUTONOMOUS_FULL_BLUE_SIDE
 *
 * This Autonomous is set to work on the the position the bottom left corner of the map.
 * In Specific, this is set to work for the blue team where there is a side between the balancing
 * board and the crypto Box.
 * Action 1:
 *  Color Sensor:
 *  Use Color Sensor to knock of the Jewel. Program loops three times
 *  and if its still is not able to color sense the robot will switch to Action 2
 * Action 2:
 * Vuforia:
 *  Robot moves to the Pictograph and senses the orientation of the cryptoBox.
 *  Checks 1 time for orientation
 * Action 3:
 *  Glyphs
 *  Catches glyph in front of robot and moves in to the specified slot of the cryptoBox.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Autonoumous BLUE SIDE", group ="Concept")

public class Autonomous_Full_Blue_Side extends LinearOpMode
{
    OpenGLMatrix lastLocation = null;
    HardwarePushbotCustom20178 robot = new HardwarePushbotCustom20178();
    EncoderMethods Encoders  = new EncoderMethods();
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor colorSensor;
    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException {


        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        robot.init(hardwareMap);
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AfprXUL/////AAAAGfY1xZ7or0lruQmUNSY4+LeBq0LGWrhvHG5HbKapX+MFc5SVVV4j2DBciNWGTmqmG7eIUk8VE30Eg+pmHdPaY1nhyMUa3dvzQqHMy4N7ddy/P63ZDPet4CjHrL2ny6oFVgv13CLUpu2mt/MiYlnPP6gcuY4Jdib8L9hOPCYeVb6gE/cnBUKV2ZhsMe5WekSJDHfIbeIX+ZecnA+mqIhrRs3u9tHXSjm5+a0NtAJMTHBI6kvpfUyU7ZyJkGCy1Yf62cPOXGH9nI7mmRvbDhSZMSQDpc0J1op1I2E3jMK8XSQG0WeMlz7iiliiADsx/Fno/8HilKlC4GJnJXPROyKmrru/OAO3ykj+8s9M0cuWaYnI";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        if (opModeIsActive()) {
            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
//            bPrevState = bCurrState;
            robot.leftArmHandle.setPosition(1.2);
            robot.rightArmHandle.setPosition(-0.61);
            robot.leftArmMotor.setPower(0.5);
            robot.rightArmMotor.setPower(0.5);
            sleep(500);
            robot.colorServo.setPosition(0.63);

            sleep(4000);
            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xFF, values));

                }
            });
            telemetry.update();
            /**
             *  Action 1:
             *  Color Sensor:
             *  Use Color Sensor to knock of the Jewel. Program loops three times
             *  and if its still is not able to color sense the robot will switch to Action 2
             */
            for(int i  =0 ; i  < 3; i++)
            {
                /*
                 * Color Sensor Jewel Red Ball Blue team:
                 * Move Color Sensor Arm  0.63 inches Down.
                 * Move Backward 2.25 inches.
                 * Move Color Sensor to initial position.
                 * Move Forward 2.25 inches.
                  */
                if (colorSensor.red() > 0 && colorSensor.blue() <= 0)
                {
                    robot.colorServo.setPosition(0.63);
                    Encoders.DriveBackward(2.25);
                    robot.colorServo.setPosition(0.0);
                    Encoders.DriveForward(2.25);
                    break;
                }
                /*
                 * Color Sensor Jewel Blue Ball Blue team:
                 * Move Color Sensor Arm  0.63 inches Down.
                 * Move Forward 2.25 inches.
                 * Move Color Sensor to initial position.
                 * Move Blue 2.25 inches.
                  */
                else if (colorSensor.blue() > 0 && colorSensor.red() <= 0)
                {
                    robot.colorServo.setPosition(0.63);
                    Encoders.DriveForward(2.25);
                    robot.colorServo.setPosition(0.0);
                    Encoders.DriveBackward(2.25);
                    break;
                }
                /*
                 * Color Sensor Not Visible:
                 * Move Color Sensor Arm to initial position.
                 * Sleep for .75 seconds
                 * Move Color Sensor Arm  0.63 inches Down.
                 * Move Backward 2.25 inches.
                 * Move Color Sensor to initial position.
                 * Sleep for 4 seconds
                  */
                else
                {
                    robot.colorServo.setPosition(0.0);
                    sleep(750);
                    robot.colorServo.setPosition(0.63);
                    sleep(4000);
                    /*
                 * Color Sensor Not Visible:
                 * If not visible 3rd times set color sensor back to initial position.
                  */
                    if ( i == 2)
                    {
                        robot.colorServo.setPosition(0.0);
                    }
                }
            }

            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.colorServo.setPosition(0.0);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
            while (opModeIsActive() && (runtime.seconds() < 0.2)) {

                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            /**
             * Action 2:
             * Vuforia:
             *  Robot moves to the Pictograph and senses the orientation of the cryptoBox.
             *  Checks 1 time for orientation
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {

               /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                /**
                 * Action 3:
                 *  Glyphs
                 *  Catches glyph in front of robot and moves in to the specified slot of the cryptoBox.
                 */
                /*
                 * VuMark Left:
                 * Move Forward 32 inches
                 * Turn Left 90
                 * Strafe 9 inches Left
                 * Open arm
                 * Drive Forward Four Inches.
                  */
                if(vuMark==RelicRecoveryVuMark.LEFT)
                {
                    Encoders.DriveForward(32.0);
                    Encoders.TurnLeft90();
                    Encoders.DriveStrafeLeft(9.0);
                    robot.rightArmHandle.setPosition(1.0);
                    robot.leftArmHandle.setPosition(0.0);
                    Encoders.DriveForward(4.0);
                }
                /*
                 * VuMark Center:
                 * Move Forward 32 inches
                 * Turn Left 90
                 * Open arm
                 * Drive Forward Four Inches.
                  */
                else if(vuMark==RelicRecoveryVuMark.CENTER)
                {
                    Encoders.DriveForward(32.0);
                    Encoders.TurnLeft90();
                    robot.rightArmHandle.setPosition(1.0);
                    robot.leftArmHandle.setPosition(0.0);
                    Encoders.DriveForward(4.0);
                }
                /*
                 * VuMark Right:
                 * Move Forward 32 inches
                 * Turn Left 90
                 * Strafe 9 inches Right
                 * Open arm
                 * Drive Forward Four Inches.
                  */
                else if(vuMark==RelicRecoveryVuMark.RIGHT)
                {
                    Encoders.DriveForward(32.0);
                    Encoders.TurnLeft90();
                    Encoders.DriveStrafeRight(9.0);
                    robot.rightArmHandle.setPosition(1.0);
                    robot.leftArmHandle.setPosition(0.0);
                    Encoders.DriveForward(4.0);
                }
            }
            /*
            * VuMark NonVisible:
            * Moves exactly Like VuMark Center.
             */
            else
            {
                Encoders.DriveForward(32.0);
                Encoders.TurnLeft90();
                robot.rightArmHandle.setPosition(1.0);
                robot.leftArmHandle.setPosition(0.0);
                Encoders.DriveForward(4.0);
            }

            telemetry.update();

        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
