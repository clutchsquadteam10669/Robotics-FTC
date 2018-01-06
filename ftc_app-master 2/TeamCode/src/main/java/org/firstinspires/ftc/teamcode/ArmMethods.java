package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by ProUser on 12/14/17.
 */
@Autonomous(name="Arm", group ="Concept")
//@Disabled
public class ArmMethods extends LinearOpMode
{
    HardwarePushbotCustom20178 robot = new HardwarePushbotCustom20178();   // Object of the Robot
    final double ARM_SPEED = 0.03; // sets rate to move servo
    double ARM_POSITION_LEFT = 0.00; // Initial Position for Arm Claw Left
    double ARM_POSITION_RIGHT = 1.00;// Initial Position for Arm Claw Right
    public void runOpMode()
    {

    }
    public void Close(double distance)
    {

    }
    public void Open(double distance)
    {

    }
    public void CloseFull()
    {
        robot.rightArmHandle.setPosition(0.0);
        robot.leftArmHandle.setPosition(1.0);

    }
    public void OpenFull()
    {
        robot.rightArmHandle.setPosition(1.0);
        robot.leftArmHandle.setPosition(0.0);
    }

}
