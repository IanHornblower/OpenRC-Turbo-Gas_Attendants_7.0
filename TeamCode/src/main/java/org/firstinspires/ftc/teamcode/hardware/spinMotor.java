package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class spinMotor {

    Robot robot;

    DcMotor left;
    DcMotor right;

    public static double speed = 0.74;

    public static double leftMult = -1;
    public static double rightMult = -1;

    public spinMotor(Robot robot) {
        this.robot = robot;
        this.right = robot.getRight();
        this.left = robot.getLeft();
    }

    public void run(boolean run) {
        if(run) {
            robot.getLeft().setPower(-speed*leftMult);
            robot.getRight().setPower(speed*rightMult);
        }
        else {
            robot.getLeft().setPower(0);
            robot.getRight().setPower(0);
        }
    }

    public void reverse(boolean run) {
        if(run) {
            robot.getLeft().setPower(speed*leftMult);
            robot.getRight().setPower(-speed*rightMult);
        }
        else {
            robot.getLeft().setPower(0);
            robot.getRight().setPower(0);
        }
    }

}
