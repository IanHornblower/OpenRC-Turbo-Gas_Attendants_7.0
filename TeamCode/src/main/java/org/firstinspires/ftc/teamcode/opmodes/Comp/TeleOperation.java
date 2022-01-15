package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;
import static org.firstinspires.ftc.teamcode.hardware.lift.*;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Comp")
public class TeleOperation extends LinearOpMode {

    public static double p = 0.007, i = 0, d = 5;
    public static double position = liftStart;
    boolean down = true;
    boolean isMoving = false;
    boolean intakeDown = true;

    private enum DRIVE {
        FIELD,
        ROBOT
    }

    public static enum LIFTSTATE {
        START,
        ONE,
        TWO,
        THREE;
    }

    public static enum TELEOPSTATE {
        START,
        PRIME,

    }

    LIFTSTATE liftPos = LIFTSTATE.START;

    DRIVE driveState = DRIVE.FIELD;

    final LIFTSTATE[] lift = {LIFTSTATE.ONE};

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(PoseStorage.autoEnd);
        double modTheta = robot.START_POSITION.heading;

        switch (MatchConfig.side) {
            case RED:
                robot.getDuck().setDirection(DcMotorSimple.Direction.REVERSE);
                modTheta = -Math.PI/2+modTheta;
                break;
            case BLUE:
                robot.getDuck().setDirection(DcMotorSimple.Direction.FORWARD);
                modTheta = Math.PI/2+modTheta;
        }

        waitForStart();

        robot.lift.startServo();

        while(opModeIsActive()) {
            Thread t1 = new Thread(() -> {
                switch (lift[0]) {
                    case ONE:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.TWO;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.THREE;
                        break;
                    case TWO:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.THREE;
                        if(gamepad2.dpad_left) lift[0]  = LIFTSTATE.ONE;
                        break;
                    case THREE:
                        sleep(200);
                        if(gamepad2.dpad_right)  lift[0] = LIFTSTATE.ONE;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.TWO;
                        break;
                }
            });

            t1.start();

            robot.updateOdometry();

            // Drive Train

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            if(driveState == DRIVE.FIELD) {
                robot.DriveTrain.driveFieldCentric(leftX, leftY, turn, modTheta);
                if(gamepad1.square) driveState = DRIVE.ROBOT;
            }
            else {
                robot.DriveTrain.setMotorPowers(leftX, leftY, turn);
                if(gamepad1.triangle) driveState = DRIVE.FIELD;
            }

            isMoving = leftX > 0.1 || leftY > 0.1;

            // Duck Motor
            robot.spinMotor.run(gamepad2.left_bumper, gamepad2.right_bumper);

            // Intake

            if(gamepad2.dpad_down)  {
                intakeDown = true;
                robot.intakeSys.regularFreightIntake();
            }

            if(gamepad2.dpad_up) {
                intakeDown = false;
                robot.intakeSys.raiseIntake();
            }

            if(gamepad1.dpad_left || gamepad1.dpad_right) robot.intakeSys.inAirIntake();

            robot.intakeSys.run(gamepad2.left_trigger, gamepad2.right_trigger, down);

            // Lift

            robot.lift.setPosition(position);

            if(gamepad2.square && !isMoving && intakeDown) {
                down = false;
                if (lift[0] == LIFTSTATE.ONE) {
                    position = liftOne;
                    robot.lift.primeServo();
                } else if (lift[0] == LIFTSTATE.TWO) {
                    position = liftTwo;
                    robot.lift.primeServo();
                } else if (lift[0] == LIFTSTATE.THREE) {
                    position = liftThree;
                    robot.lift.primeServo();
                }
            }

            if(gamepad2.dpad_down) {
                robot.lift.setLiftPID(new MiniPID(0.003, i, d));
            }
            else robot.lift.setLiftPID(new MiniPID(p, i, d));

            if(gamepad2.dpad_down) {
                robot.lift.setLiftPID(new MiniPID(0.003, i, d));
            }
            else robot.lift.setLiftPID(new MiniPID(p, i, d));

            if(gamepad2.circle && !down && !isMoving) {
                robot.lift.drop();
            }

            if(gamepad2.triangle && !isMoving && intakeDown) {
                down = false;
                position = liftThree;
                robot.lift.primeServo();
            }

            if(gamepad2.cross && intakeDown) {
                down = true;
                robot.lift.startServo();
                position = liftStart;
            }

            //telemetry.addData("End Auto Pos", PoseStorage.autoEnd.toString());
            //telemetry.addData("Current Pos", robot.pos.toString());
            //telemetry.addData("IMU ANGLE", robot.IMU.getIMUHeading() + "DEGREES: " + Math.toDegrees(robot.IMU.getIMUHeading()));
            //telemetry.addData("Robot ANGLE", robot.pos.heading + "DEGREES: " + Math.toDegrees(robot.pos.heading));

            telemetry.addData("Lift Level", lift[0].toString());
            telemetry.addData("Drive State", driveState.toString());
            telemetry.addData("Side", MatchConfig.side.toString());
            telemetry.update();
        }
    }
}
