package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.nio.charset.MalformedInputException;

@Config
public class lift {

    DcMotor lift;
    Servo boxServo;
    MiniPID liftPID;
    Robot robot;

    public static double P = 0.02, I = 0, D = 0;

    public static double threshold = 10;

    public static double liftStart = 0;
    public static double liftPrimed = 100;
    public static double liftOne = 0;
    public static double liftTwo = 0;
    public static double liftThree = 0;

    public static double servoStart = 0;
    public static double servoPrimed = 0;
    public static double servoDropped = 0;

    public enum LIFT {
        START,
        PRIMED,
        D1,
        D2,
        D3
    }

    public enum SERVO {
        START,
        PRIMED,
        DROPPED
    }

    LIFT liftState;
    SERVO servoState;

    public lift(Robot robot) {
        this.lift = robot.getLift();
        this.boxServo = robot.getBoxServo();
        this.robot = robot;
        liftPID = new MiniPID(P, I, D);
        liftState = LIFT.START;
        servoState = SERVO.START;
    }
    
    public void setPosition(double position) {
        if(Math.abs(robot.getLift().getCurrentPosition() - position) < threshold) {
            liftPID.setSetpoint(position);
            liftPID.setError(position - robot.getLift().getCurrentPosition());
            robot.getLift().setPower(liftPID.getOutput());
        }
        else robot.getLeftEncoder().setPower(0);
    }

    public void prime() {
        if (liftState == LIFT.START) {
            setPosition(liftPrimed);
            if (Math.abs(robot.getLift().getCurrentPosition() - liftPrimed) < threshold) {
                boxServo.setPosition(servoPrimed);
                liftState = LIFT.PRIMED;
            }
        } else {
            setPosition(liftPrimed);
            boxServo.setPosition(servoPrimed);
            if(Math.abs(robot.getLift().getCurrentPosition() - liftPrimed) < threshold) liftState = LIFT.PRIMED;
        }
    }
    public void retract() {
        switch (liftState) {
            case START:
                liftState = LIFT.START;
                break;
            case PRIMED:
                boxServo.setPosition(servoStart);
                setPosition(liftStart);
        }
    }
}