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

import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Comp")
public class TeleOperation extends LinearOpMode {

    private enum DRIVE {
        FIELD,
        ROBOT
    }

    public static enum LIFTSTATE {
        ONE,
        TWO,
        THREE;
    }

    public static enum TELEOPSTATE {
        START,
        PRIME,

    }

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
                modTheta -= Math.PI;
                break;
            case BLUE:
                robot.getDuck().setDirection(DcMotorSimple.Direction.FORWARD);
                modTheta += Math.PI;
        }

        waitForStart();

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

            // Duck Motor
            robot.spinMotor.run(gamepad2.left_bumper, gamepad2.right_bumper);

            // Intake

            if(gamepad1.dpad_down) robot.intakeSys.regularFreightIntake();

            if(gamepad1.dpad_up) robot.intakeSys.raiseIntake();

            if(gamepad1.dpad_left || gamepad1.dpad_right) robot.intakeSys.inAirIntake();

            robot.intakeSys.run(gamepad2.left_trigger > 0.1, gamepad2.right_trigger > 0.1);

            // Lift

            if(gamepad2.square) {
                switch (lift[0]) {
                    case ONE:

                    case TWO:
                        System.out.println(lift[0].toString());
                    case THREE:
                        System.out.println(lift[0].toString());
                }
            }

            if(gamepad2.dpad_up) {
                robot.lift.prime();
            }
            else if(gamepad2.dpad_down) {
                robot.lift.retract();
            }

            if(gamepad2.cross) {
                robot.lift.drop();
            }

            telemetry.addData("Lift Level", lift[0].toString());
            telemetry.addData("Drive State", driveState.toString());
            telemetry.update();
        }
    }
}
