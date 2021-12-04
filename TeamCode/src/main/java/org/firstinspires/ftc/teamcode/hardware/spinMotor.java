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
        this.right = robot.getDuckRight();
        this.left = robot.getDuckLeft();
    }

    public void run(boolean run) {
        if(run) {
            robot.getDuckLeft().setPower(-speed*leftMult);
            robot.getDuckRight().setPower(speed*rightMult);
        }
        else {
            robot.getDuckLeft().setPower(0);
            robot.getDuckRight().setPower(0);
        }
    }

    public void reverse(boolean run) {
        if(run) {
            robot.getDuckLeft().setPower(speed*leftMult);
            robot.getDuckRight().setPower(-speed*rightMult);
        }
        else {
            robot.getDuckLeft().setPower(0);
            robot.getDuckRight().setPower(0);
        }
    }

}
