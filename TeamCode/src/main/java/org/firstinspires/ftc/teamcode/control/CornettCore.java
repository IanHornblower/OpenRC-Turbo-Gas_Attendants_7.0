package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import static org.firstinspires.ftc.teamcode.control.Coefficients.*;
import static org.firstinspires.ftc.teamcode.util.Init.init;

@Config
public class CornettCore extends OpMode {

    //TODO: Make sure it keeps itself centered when it has no rotation

    private Robot robot;

    public enum DIRECTION {FORWARD, BACKWARD}

    Coefficients PIDEx = new Coefficients();

    MiniPID defaultTurnPID = new MiniPID(turnKp, turnKi, turnKd);

    public double defaultTurnOutputMultiplier = 1;

    public static double xP = 0.17;
    public static double yP = 0.17;
    public static double headingP = turnKp; // 0.04

    public static double xI = 0;
    public static double yI = 0;
    public static double headingI = turnKi; // 0.01

    public static double xD = 0;
    public static double yD = 0;
    public static double headingD = turnKd; // 0.01

    MiniPID defaultXPID = new MiniPID(xP, xI,xD);
    MiniPID defaultYPID = new MiniPID(yP, yI,yD);
    MiniPID defaultHeadingPID = new MiniPID(headingP, headingI, headingD);

    public static double defaultXControlPointMultiplier = 1;
    public static double defaultYControlPointMultiplier = 1;
    public static double defaultHeadingControlPointMultiplier = 1;

    private double zero = 1e-9;
    boolean init = false;
    double distance = 0;
    double angleDistance = 0;

    public static double Dp = 0.07;
    public static double ACp = 1;

    public double output = 0, direction = 0, turnPIDOutput = 0;

    double xPIDOutput, yPIDOutput, headingPIDOutput;

    public CornettCore(Robot robot) {
        this.robot = robot;
    }

    public void tuneTrackWidthIMU (double heading, double direction) {
        robot.updateAccumulatedHeading();
        /*
        MiniPID turnPID = defaultTurnPID;

        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Math.abs(heading - Math.toRadians(robot.IMU.getAccumulatedHeadingInDegrees())));

        turnPIDOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.IMU.getAccumulatedHeadingInDegrees()));
        output = direction * turnPIDOutput * defaultTurnOutputMultiplier;

         */

        output = PIDEx.turn.calculate(heading, Math.toRadians(robot.IMU.getAccumulatedHeadingInDegrees()));

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotateRaw(double heading, MiniPID turnPID, double outputMultiplier) {
        robot.updateOdometry();

        turnPID.setSetpoint(heading);
        turnPID.setOutputLimits(-1, 1);

        turnPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);
        turnPIDOutput = turnPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));
        output = direction * turnPIDOutput * outputMultiplier;

        //output = PIDEx.turn.calculate(heading, robot.IMU.getIMUHeading());

        robot.DriveTrain.setMotorPowers(0, 0, output);
    }

    public void rotate(double heading) {
        rotateRaw(heading, defaultTurnPID, defaultTurnOutputMultiplier);
    }

    public void rotateSyncRaw(double heading, double anglePrecision, MiniPID turnPID, double turnOutputMultiplier) {
        distance = Curve.getShortestDistance(heading, anglePrecision);

        do {
            robot.updateOdometry();
            distance = Curve.getShortestDistance(robot.pos.getHeading(), heading);
            rotateRaw(heading, turnPID, turnOutputMultiplier);
        } while(distance > anglePrecision);
    }

    public void rotateSync(double heading, double anglePrecision) {
        rotateSyncRaw(heading, anglePrecision, defaultTurnPID, defaultTurnOutputMultiplier);
    }

    public void runToPositionRaw(double x, double y, double heading, MiniPID xPID,
                                 MiniPID yPID, MiniPID headingPID,
                                 double xControlPointMultiplier, double yControlPointMultiplier, double headingControlPointMultiplier)
    {
        /*

        xPID.setSetpoint(x);
        yPID.setSetpoint(y);
        headingPID.setSetpoint(heading);

        xPID.setOutputLimits(-1, 1);
        yPID.setOutputLimits(-1, 1);
        headingPID.setOutputLimits(-1, 1);

        xPID.setError(Math.abs(x-robot.pos.x));
        yPID.setError(Math.abs(y-robot.pos.y));
        headingPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        xPIDOutput = xPID.getOutput(robot.pos.x);
        yPIDOutput = yPID.getOutput(robot.pos.y);
        headingPIDOutput = headingPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));

         */

        double theta = Curve.getAngle(robot.pos, new Point(x, y));

        double max = Math.max(Math.sin(theta), Math.cos(theta));

        double xRawPower = Math.abs(Math.cos(theta) * max);
        double yRawPower = Math.abs(Math.sin(theta) * max);

        xPIDOutput = PIDEx.x.calculate(x, robot.pos.x);
        yPIDOutput = PIDEx.y.calculate(y, robot.pos.y);

        headingPID.setSetpoint(heading);
        headingPID.setOutputLimits(-1, 1);

        headingPID.setError(Curve.getShortestDistance(robot.pos.getHeading(), heading));

        direction = Curve.getDirection(robot.pos.getHeading(), heading);
        turnPIDOutput = headingPID.getOutput(AngleUtil.deNormalizeAngle(robot.pos.getHeading()));

        double xControlPoint = 1 * xPIDOutput * xControlPointMultiplier;
        double yControlPoint = 1 * yPIDOutput * yControlPointMultiplier;
        double headingControlPoint = direction * turnPIDOutput * headingControlPointMultiplier;

        robot.DriveTrain.driveFieldCentric(xControlPoint, yControlPoint, headingControlPoint);

        //telemetry = robot.dashboard.getTelemetry();

        //telemetry.addData("robot x", robot.pos.x);
        //telemetry.addData("robot y", robot.pos.y);
        //telemetry.addData("x", x);
        //telemetry.addData("y", y);
        //telemetry.addData("x error", x - robot.pos.x);
        //telemetry.addData("y error", y - robot.pos.y);
        //telemetry.addData("Theta", Math.toDegrees(theta));
        //telemetry.addData("Raw Powers: x = ", xRawPower + "\t y = " + yRawPower);
        //telemetry.addData("PID outputs: x = ", xPIDOutput + "\t y = " + yPIDOutput);
        //telemetry.addData("Control Points: x = ", xControlPoint + "\t y = " + yControlPoint);
        //telemetry.update();
    }

    public void runToPosition(double x, double y, double heading) throws InterruptedException {
        runToPositionRaw(
                x, y, heading,
                defaultXPID, defaultYPID, defaultHeadingPID,
                defaultXControlPointMultiplier, defaultYControlPointMultiplier, defaultHeadingControlPointMultiplier);
    }

    public synchronized void runToPositionSyncRaw(double x, double y, double heading, double allowableDistanceError,
                                                  MiniPID xPID, MiniPID yPID, MiniPID headingPID,
                                                  double xControlPointMultiplier, double yControlPointMultiplier, double headingControlPointMultiplier) throws InterruptedException {
        distance = robot.pos.getDistanceFrom(new Point(x, y));
        angleDistance = Curve.getShortestDistance(heading, robot.pos.getHeading());
        do {
            robot.updateOdometry();
            distance = robot.pos.getDistanceFrom(new Point(x, y));
            angleDistance = Curve.getShortestDistance(heading, robot.pos.getHeading());
            runToPositionRaw(
                    x, y, heading,
                    xPID, yPID, headingPID,
                    xControlPointMultiplier, yControlPointMultiplier, headingControlPointMultiplier);
        } while(distance > allowableDistanceError); //  ||  angleDistance > Math.toRadians(allowableDistanceError));
    }

    public synchronized void runToPositionSync(double x, double y, double heading, double allowableDistanceError) throws InterruptedException {
        runToPositionSyncRaw(
                x, y, heading, allowableDistanceError,
                defaultXPID, defaultYPID, defaultHeadingPID,
                defaultXControlPointMultiplier, defaultYControlPointMultiplier, defaultHeadingControlPointMultiplier);
    }

    double t = 0, f = 0;

    public void setDifMotorForward(double targetX, double targetY) {
        robot.updateOdometry();

        double xError = targetX - robot.pos.x;
        double yError = targetY - robot.pos.y;
        double theta = Math.atan2(yError,xError);

        double distance = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

        f = 0 - distance * Dp;  // Could be -distance but Zero is there to model proper P-Loop
        t = AngleUtil.angleWrap(theta - robot.pos.getHeading()) * ACp;

        double left = f + t;
        double right = f - t;

        robot.DriveTrain.setMotorPowers(left, right);
    }

    public void setDifMotorReverse(double targetX, double targetY) {
        robot.updateOdometry();

        double xError = targetX - robot.pos.x;
        double yError = targetY - robot.pos.y;
        double theta = Math.atan2(-yError,-xError);

        double distance = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

        f = 0 - distance * Dp;  // Could be -distance but Zero is there to model proper P-Loop
        t = AngleUtil.angleWrap(theta - robot.pos.getHeading()) * ACp;

        double left = -f + t;
        double right = -f - t;

        robot.DriveTrain.setMotorPowers(left, right);
    }

    public void differentialRunToPosition(DIRECTION direction, Point pos) {
        switch (direction) {
            case FORWARD:
                setDifMotorForward(pos.x, pos.y);
                break;
            case BACKWARD:
                setDifMotorReverse(pos.x, pos.y);
        }
    }

    public void runEncoders(double power, double distance) {
        double left = robot.getLeftEncoder().getCurrentPosition();

        double state = 0;
        if(distance > state) {
           state = Math.abs(robot.getLeftEncoder().getCurrentPosition() - left);
           robot.DriveTrain.setMotorPowers(power, power);
            }
        }

    public void runTime(double power, double time) {
        new Thread(()-> {
            robot.DriveTrain.setMotorPowers(power, power);
            try {
                Thread.sleep((long)time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.DriveTrain.stopDrive();
        }).start();
    }

    public void runTimeSync(double power, double time) {
            robot.DriveTrain.setMotorPowers(power, power);
            try {
                Thread.sleep((long)time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.DriveTrain.stopDrive();
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}