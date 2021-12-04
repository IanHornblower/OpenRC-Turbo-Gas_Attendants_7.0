package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    Robot robot;
    DcMotor intake;
    Servo intakeServo;

    public static double ON = 1;

    public static double UP = 1;
    public static double RAISED = 0.7;
    public static double REGULAR_DOWN = 0.4;
    public static double DUCK_DOWN = 0.3;


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

    public void stopIntake() {
        setIntakePower(0);
    }

    public void raiseIntake() {
        robot.getIntakeServo().setPosition(UP);
    }

    public void inAirIntake() {
        robot.getIntakeServo().setPosition(RAISED);
    }

    public void regularFreightIntake() {
        robot.getIntakeServo().setPosition(REGULAR_DOWN);
    }

    public void duckIntake() {
        robot.getIntakeServo().setPosition(DUCK_DOWN);
    }

}
