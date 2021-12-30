package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;

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
            robot.updateAccumulatedHeading();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            robot.DriveTrain.driveFieldCentric(0, leftY, turn);

            if(gamepad1.square) {
                robot.getFrontLeft().setPower(1);
            }

            else if(gamepad1.triangle) {
                robot.getFrontRight().setPower(1);
            }

            else if(gamepad1.circle) {
                robot.getBackRight().setPower(1);
            }

            else if(gamepad1.cross) {
                robot.getBackLeft().setPower(1);
            }

            else {
                robot.stopDrive();
            }

            //telemetry.addData("\nXYH", robot.pos.toString());
            //telemetry.update();
        }
    }
}
