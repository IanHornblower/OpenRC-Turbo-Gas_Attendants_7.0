package org.firstinspires.ftc.teamcode.opmodes.TrajectoryStuff;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;
@Config
@Autonomous(name = "PID Testing", group = "Traj")
public class TrajectoryAutoTest extends LinearOpMode {

    public static boolean turn = false;
    public static boolean loop = false;
    public static double interval = 0;
    public static double angle = 90;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Create Trajectories and Function Lists

        robot.intakeSys.regularFreightIntake();
        robot.lift.startServo();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if (turn) {
                motionProfile.rotateSync(Math.toRadians(angle), Math.toRadians(1));
            }

            else {
                motionProfile.runToPositionSync(24, 24, Math.toRadians(angle), 1);
            }

            if(!loop) stop();
            sleep((long)interval);


            Trajectory joe = new Trajectory(robot, robot.START_POSITION);

            joe.addWaypoint(new Point(10, 10));
            joe.addWaypoint(new Point(24, 24));
        }

    }
}
