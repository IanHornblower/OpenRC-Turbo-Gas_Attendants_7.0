package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Robot;
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
@TeleOp(name = "RedSide TeleOp", group = "Comp")
public class RedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(PoseStorage.autoEnd);

        waitForStart();

        while(opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            robot.updateOdometry();

            // Drive Train

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            robot.DriveTrain.driveFieldCentric(leftX, leftY, turn);

            // Duck Motor

            robot.spinMotor.run(gamepad2.right_bumper);
            robot.spinMotor.reverse(gamepad2.left_bumper);

            // Lift System

            if(gamepad2.dpad_down) {
                robot.lift.setServoState(lift.SERVOSTATE.ONE);
            }

            if(gamepad2.dpad_right) {
                robot.lift.setServoState(lift.SERVOSTATE.TWO);
            }

            if(gamepad2.dpad_up) {
                robot.lift.setServoState(lift.SERVOSTATE.THREE);
            }

            if(gamepad2.dpad_left) {
                robot.lift.setServoState(lift.SERVOSTATE.INTAKE_LEVEL);
            }

            if(gamepad2.y) {
                robot.lift.primeLift();
            }

            if(gamepad2.b) {
                robot.lift.scoreFreight();
            }

            if(gamepad2.a) {
                robot.lift.returnLift();
            }

            // Intake System

            if(gamepad2.dpad_down) {
                robot.intakeSys.regularFreightIntake();
            }

            if(gamepad2.dpad_up) {
                robot.intakeSys.raiseIntake();
            }

            if(gamepad2.dpad_left) {
                robot.intakeSys.raiseIntake();
            }

            if(gamepad2.dpad_right) {
                robot.intakeSys.duckIntake();
            }

            robot.getLift().setPower(gamepad2.left_stick_y);

            robot.intakeSys.reverse(gamepad2.left_trigger > 0.1);

            robot.intakeSys.run(gamepad2.right_trigger > 0.1);

            telemetry.addData("Lift", robot.lift.state.toString());

            telemetry.addData("Lift Motor Enc", robot.getLift().getCurrentPosition());

            telemetry.addData("\nXYH", robot.pos.toString());
            telemetry.update();
        }
    }
}
