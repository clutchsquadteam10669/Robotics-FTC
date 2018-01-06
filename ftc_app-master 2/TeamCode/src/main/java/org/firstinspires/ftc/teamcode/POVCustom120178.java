package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * POVCUSTOM20178: Driving Code for 2017-2018 Relic Recovery FTC
 * <p>
 * Contains Code for Driving the Robot and Controlling the Arm in Teleop
 * <p>
 * Robot Code has two modes Manual Mode and Mecanum Drive Mode,
 * In Mecanum Drive mode you can move at any angle and the robot is controlled through the joystick
 * In Manual Drive Mode the robot moves using DPAD, Left and Right bumber and X, A and B.
 */

@TeleOp(name = "Driver Controlled Period Final", group = "Linear Opmode")


public class POVCustom120178 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotCustom20178 robot = new HardwarePushbotCustom20178();   // Object of the Robot
    final double ARM_SPEED = 0.03; // sets rate to move servo
    double ARM_POSITION_LEFT = 0.00; // Initial Position for Arm Claw Left
    double ARM_POSITION_RIGHT = 1.00;// Initial Position for Arm Claw Right
    double Forward = 0.40; // Power Robot uses for Forward direction of Robot.
    double Backward = -0.40; // Power Robot uses for Backward direction of Robot.
    boolean shouldMecanumDrive = false; // Checks if we are using Manual drive or Mecanum Drive
    EncoderMethods Encoders = new EncoderMethods();
    @Override
    public void runOpMode() {

        // double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        /**
         * Robot Control Code:
         * Gamepad 1: Controls Driving
         * Gampepad 2: Controls Arm
         */
        while (opModeIsActive()) {


            /**
             * Manual Version Driving Code:
             * DPAD_UP: Robot move Forward
             * DPAD_DOWN: Robot move Backward
             * DPAD_LEFT: Strafe Left
             * DPAD_Right: Strafe Right
             * LEFT_BUMPER: Turn Left
             * RIGHT_BUMPER: Turn Right
             * A: 180 degree Turn
             * X: Turn Left 90 degrees
             * B: Turn Right 90 degrees
             */
            if (gamepad1.dpad_up) // Move Forward
            {
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Forward);
                robot.leftBackMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Forward);
            }
            else if (gamepad1.dpad_down) // Move Backward
            {
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Backward);
                robot.leftBackMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Backward);
            }
            else if (gamepad1.dpad_left) // Strafe Left
            {
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Forward);
                robot.leftBackMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Backward);
            }
            else if (gamepad1.dpad_right)// Strafe Right
            {
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Backward);
                robot.leftBackMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Forward);
            }
            else if (gamepad1.left_bumper) // Turn Left
            {
                robot.leftBackMotor.setPower(Backward);
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Forward);
            }
            else if (gamepad1.right_bumper) // Turn Right
            {
                robot.leftBackMotor.setPower(Forward);
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Backward);
            }
            else if (gamepad1.x) // Turn Left Exactly 90 degrees.
            {
                Encoders.TurnLeft90();
            }
            else if (gamepad1.b) // Turn Right Exactly 90 degrees.
            {
                Encoders.TurnRight90();
            }
            else if (gamepad1.a) // Turn 180 degrees
            {
                Encoders.Turn180();
            }
            else
            {
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
            }


            /**
             * Starting Arm Code:
             * X: Close Claws
             * Y: Opens Claws
             * DPAD_UP: Extend Arms
             * DPAD_DOWN: Retracts Arms
             * LEFT_BUMPER: Pulls Arm up
             * RIGHT_BUMPER: DROPS ARM down
             */
            if (gamepad2.x) // Close Claws
            {
                ARM_POSITION_RIGHT -= ARM_SPEED;
                robot.rightArmHandle.setPosition(ARM_POSITION_RIGHT);
                ARM_POSITION_LEFT += ARM_SPEED;
                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
            }
            else if (gamepad2.y)// Opens Claws
            {

                ARM_POSITION_RIGHT += ARM_SPEED;
                robot.rightArmHandle.setPosition(ARM_POSITION_RIGHT);
                ARM_POSITION_LEFT -= ARM_SPEED;
                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
            }

            if (gamepad2.dpad_up) // Extend Arms
            {
                robot.bottomArmMotor.setPower(robot.RACK_AND_PINON);
            }
            else if (gamepad2.dpad_down) // Retracts Arms
            {
                robot.bottomArmMotor.setPower(robot.RACK_AND_PINON_REVERSE);
            }
            else
            {
                robot.bottomArmMotor.setPower(0.0);
            }

            if (gamepad2.left_bumper) // Pulls Arm up
            {
                robot.leftArmMotor.setPower(robot.ARM_FORWARD_POWER);
                robot.rightArmMotor.setPower(robot.ARM_FORWARD_POWER);
            }
            else if (gamepad2.right_bumper) // Drops Arms Down
            {
                robot.leftArmMotor.setPower(robot.ARM_REVERSE_POWER);
                robot.rightArmMotor.setPower(robot.ARM_REVERSE_POWER);
            }
            else
            {
                robot.leftArmMotor.setPower(0.0);
                robot.rightArmMotor.setPower(0.0);
                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);
                idle();
            }
        }
    }
}


