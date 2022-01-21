package org.firstinspires.ftc.teamcode.opmodes.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

@Config
@TeleOp(name = "TunePID", group = "Tuning")
public class TunePID extends LinearOpMode {

    public enum type{ROTATIONAL, DIRECTIONAL, BOTH, DIFFY}

    type PIDSelector;


    public static double scaleBox = 3;
    public static long waitPeriod = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        Trajectory joe = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(0, 16));
        joe.addWaypoint(new Point(12, 24));
        joe.addWaypoint(new Point(12, 45));

        joe.retrace();

        boolean ran = false;
        telemetry.addLine("Press X for tuning heading PID\nPress B for Tuning xy PID\nPress Y for both\nPress A for Tuning Diffy");
        telemetry.update();

        do {
            if (gamepad1.x || gamepad2.x) {
                ran = true;
                PIDSelector = type.ROTATIONAL;
            }

            if (gamepad1.b || gamepad2.b) {
                ran = true;
                PIDSelector = type.DIRECTIONAL;
            }

            if (gamepad1.y || gamepad2.y || gamepad1.a || gamepad2.a) {
                ran = true;
                PIDSelector = type.BOTH;
            }

        } while(!ran);

        telemetry.clear();

        telemetry.addData("Tuning", PIDSelector);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            CornettCore motionProfile = new CornettCore(robot);

            switch (PIDSelector) {
                case ROTATIONAL:
                    robot.updateOdometry();

                    motionProfile.rotateSync(Math.toRadians(0), Math.toRadians(0.5));

                    sleep(waitPeriod);

                    motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(0.5));
                    //break;
                case DIRECTIONAL:
                    robot.updateOdometry();

                    motionProfile.runToPositionSync(10*scaleBox, 0, Math.toRadians(90), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(10*scaleBox, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(0, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(0, 0, Math.toRadians(90), 0.5);
                    sleep(waitPeriod);

                case DIFFY:
                    robot.updateOdometry();

                    joe.retrace().followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);

                    sleep(waitPeriod);

                    joe.retrace().followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 8, 1);

                    sleep(waitPeriod);

                case BOTH:
                    robot.updateOdometry();

                    motionProfile.runToPositionSync(10*scaleBox, 0, Math.toRadians(0), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(10*scaleBox, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(0, 10*scaleBox, Math.toRadians(180), 0.5);
                    sleep(waitPeriod);
                    motionProfile.runToPositionSync(0, 0, Math.toRadians(270), 0.5);
                    sleep(waitPeriod);
                    break;

                default:
                    telemetry.addLine("Well something is fucked?");
                    telemetry.addLine("Prolly ur enums dumbfuck");
                    telemetry.update();

                    PIDSelector = type.ROTATIONAL;
            }
        }
    }
}
