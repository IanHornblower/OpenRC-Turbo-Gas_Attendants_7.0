package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.nio.charset.MalformedInputException;

@Config
public class liftMotor {

    DcMotor lift;
    MiniPID liftPID;
    Robot robot;

    public static double P = 1, I = 0, D = 0;

    public liftMotor(Robot robot) {
        this.lift = robot.getLift();
        this.robot = robot;
        liftPID = new MiniPID(P, I, D);
    }
    
    public void setPosition(double position) {
        //do {
            liftPID.setSetpoint(position);

            liftPID.setError(position - robot.getLift().getCurrentPosition());

            robot.getLift().setPower(-liftPID.getOutput());

            
        //} while(position != lift.getCurrentPosition());
        
    }
    
}