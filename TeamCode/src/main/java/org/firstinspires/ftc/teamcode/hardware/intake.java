package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.CallSuper;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class intake {

    Robot robot;
    DcMotor intake;
    Servo intakeServo;

    public static double ON = -1;

    public static double UP = 0.0;
    public static double RAISED = 0.2;
    public static double REGULAR_DOWN = 0.4;


    public intake(Robot robot) {
        this.robot = robot;
        this.intake = robot.getIntake();
        this.intakeServo = robot.getIntakeServo();
    }

    public void setIntakePower(double power) {
        robot.getIntake().setPower(power);
    }

    public void startIntake() {
        setIntakePower(ON);
    }

    public void reverseIntake() {
        setIntakePower(-ON);
    }

    public void stopIntake() {
        setIntakePower(0);
    }

    public void run(boolean run) {
        if(run) {
            if(robot.getIntake().getDirection() == DcMotorSimple.Direction.REVERSE) {
                robot.getIntake().setDirection(DcMotorSimple.Direction.FORWARD);
            }
            startIntake();
        }
        else {
            stopIntake();
        }
    }
    public void reverse(boolean run) {
        if(run) {
            if(robot.getIntake().getDirection() == DcMotorSimple.Direction.FORWARD) {
                robot.getIntake().setDirection(DcMotorSimple.Direction.REVERSE);
            }
            startIntake();
        }
        else {
            stopIntake();
        }
    }

    public void raiseIntake() {
        intakeServo.setPosition(UP);
    }


    public void inAirIntake() {
        robot.getIntakeServo().setPosition(RAISED);
    }

    public void regularFreightIntake() {
        robot.getIntakeServo().setPosition(REGULAR_DOWN);
    }
}
