package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            robot.DriveTrain.setMotorPowers(leftX, leftY, turn);

            // Duck Motor

            // Intake

            // Servo Linkage

            if(gamepad2.dpad_down) robot.intakeSys.regularFreightIntake();

            if(gamepad2.dpad_up) robot.intakeSys.raiseIntake();

            if(gamepad2.dpad_left || gamepad2.dpad_right) robot.intakeSys.inAirIntake();

            // Set Intake Power

            robot.intakeSys.setIntakePower(gamepad2.right_trigger*1e+25);
            robot.intakeSys.setIntakePower(-gamepad2.left_trigger*1e+25);

            if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger < 0.1) {
                robot.intakeSys.setIntakePower(0);
            }

            // Other

            // Telemetry

            telemetry.addData("Lift", robot.lift.state.toString());
            telemetry.addData("Lift Motor Enc", robot.getLift().getCurrentPosition());
            telemetry.addData("\nXYH", robot.pos.toString());
            telemetry.update();
        }
    }
}
