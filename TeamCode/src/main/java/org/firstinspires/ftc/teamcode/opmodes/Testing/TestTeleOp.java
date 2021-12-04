package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;

@Disabled
@Config
@TeleOp(name = "Testing OpMode", group = "Testing")
public class TestTeleOp extends LinearOpMode {

    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        while(opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();

            Field field = new Field(packet);

            robot.lift.setPosition(position);

            robot.updateOdometry();
            robot.updateVelocity();
            robot.updateAcumulatedHeading();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            robot.DriveTrain.driveFieldCentric(leftX, leftY, turn);

            field.createCircularRobot(robot.pos);

            dashboard.sendTelemetryPacket(field.getPacket());

            //telemetry.addData("Lift Pos", robot.getLift().getCurrentPosition());

            //robot.lift.setPosition(gamepad1.left_trigger);
            robot.getLift().setPower(gamepad2.right_stick_y);

            //telemetry.addData("LeftX:", gamepad1.left_stick_x + "  LeftY: " + gamepad1.left_stick_y);
            //telemetry.addData("RightX", gamepad1.right_stick_x);

            //telemetry.addData("\nXYH", robot.pos.toString());
            //telemetry.update();
        }
    }
}
