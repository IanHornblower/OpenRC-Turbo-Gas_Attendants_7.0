package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.control.Coefficients.liftKf;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

import android.util.Log;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.control.Coefficients;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.nio.charset.MalformedInputException;

@Config
public class lift {

    DcMotor lift;
    Servo boxServo;
    MiniPID liftPID;
    Robot robot;

    public static double power = 0.4;
    public static double threshold = 20;

    public static double liftStart = 20;
    public static double liftOne = 300;
    public static double liftTwo = 600;
    public static double liftThree = 1020;
    public static double liftPrimed = liftThree;

    public static double servoStart = 0.81;
    public static double servoPrimed = 0.72;
    public static double servoDropped = 0.37;
    public static double servoSemiDrop = 0.43;

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
        //liftPID = new MiniPID(P, I, D);
        liftState = LIFT.START;
        servoState = SERVO.START;
    }

    public void setLiftPID(MiniPID pid) {
        this.liftPID = pid;
    }

    public void drop() {
        boxServo.setPosition(servoDropped);
    }

    public void primeServo() {
        boxServo.setPosition(servoPrimed);
    }

    public void lowerServo() {
        boxServo.setPosition(servoSemiDrop);
    }

    public void startServo() {
        boxServo.setPosition(servoStart);
    }
    
    public void setPosition(double position) {
        /*
        if(Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            setRawPosition(position);
        }
        else robot.getLift().setPower(0.0);

         */


        // Undo Later
        setRawPosition(position, power, threshold);
    }

    public void setRawPosition(double position, double power, double tolerance) {
        /*
        double liftOutput = robot.PIDEx.lift.calculate(position, robot.getLift().getCurrentPosition());

        robot.getLift().setPower(-liftOutput*liftKf);

         */

        if(position - robot.getLift().getCurrentPosition() > 0 && Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            robot.getLift().setPower(-power);
        }
        else if(position - robot.getLift().getCurrentPosition() < 0 && Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            robot.getLift().setPower(power);
        }
        else {
            robot.getLift().setPower(0.0);
        }


    }

    public void SyncSetPosition(double position) {
        while(Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            setPosition(position);
        }
        robot.getLift().setPower(0.0);
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
                break;
            case PRIMED:
                boxServo.setPosition(servoStart);
                setPosition(liftStart);
            default:
                boxServo.setPosition(servoStart);
                setPosition(liftStart);
        }
    }

    public void prime(LIFT state) {
        Thread t1 = new Thread(()-> {
            switch(state) {
                case D1:
                    setOne();
                    boxServo.setPosition(servoPrimed);
                    break;

                case D2:
                    setTwo();
                    boxServo.setPosition(servoPrimed);
                    break;

                case D3:
                    setThree();
                    boxServo.setPosition(servoPrimed);
                    break;
            }
        });
    }

    public void setOne() {
        //while(Math.abs(lift.getCurrentPosition() - liftOne) > threshold) {
            setPosition(liftOne);
        //}
    }

    public void setTwo() {
        //while(Math.abs(lift.getCurrentPosition() - liftTwo) > threshold) {
            setPosition(liftTwo);
        //}
    }

    public void setThree() {
        //while(Math.abs(lift.getCurrentPosition() - liftThree) > threshold) {
            setPosition(liftThree);
        //}
    }

    public void setStart() {
        //while(Math.abs(lift.getCurrentPosition() - liftStart) > threshold) {
            setPosition(liftStart);
        //}
    }

    public void doAuto(FreightFrenzyCamera.position location) throws InterruptedException {
        switch (location) {
            case A:
                setOne();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
            case B:
                setTwo();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
            case C:
                setThree();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
        }
    }
}