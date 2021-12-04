package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public static double P = 0.002, I = 0, D = 0;

    public static double threshold = 20;

    public static double liftUP = 0;
    public static double liftOne = 0;
    public static double liftTwo = 0;
    public static double liftThree = 0;

    public static double lowest = 0;
    public static double levelOne = 0.3;
    public static double levelTwo = 0.5;
    public static double levelThree = 0.8;

    public enum SERVOSTATE {INTAKE_LEVEL, ONE, TWO, THREE};

    SERVOSTATE state;

    public lift(Robot robot) {
        this.lift = robot.getLift();
        this.boxServo = robot.getBoxServo();
        this.robot = robot;
        state = SERVOSTATE.INTAKE_LEVEL;
        liftPID = new MiniPID(P, I, D);
    }
    
    public void setPosition(double position) {
        //do {
            liftPID.setSetpoint(position);

            liftPID.setError(position - robot.getLift().getCurrentPosition());

            robot.getLift().setPower(-liftPID.getOutput());

            
        //} while(position != lift.getCurrentPosition());
        
    }

    public void setServoPosition() {
        switch (state) {
            case INTAKE_LEVEL:
                robot.getBoxServo().setPosition(lowest);
                break;
            case ONE:
                robot.getBoxServo().setPosition(levelOne);
                break;
            case TWO:
                robot.getBoxServo().setPosition(levelTwo);
                break;
            case THREE:
                robot.getBoxServo().setPosition(levelThree);
                break;
        }
    }

    public void setServoState(SERVOSTATE state) {
        this.state = state;
    }

    public SERVOSTATE getState() {
        return state;
    }

    public void setToFirstLevel() { // Add More when ready :)
        setServoState(SERVOSTATE.ONE);
        setPosition(liftUP);
        if(Math.abs(robot.getLift().getCurrentPosition() - liftUP) < threshold) {
            setServoPosition();
        }
        setPosition(liftOne);
    }
}