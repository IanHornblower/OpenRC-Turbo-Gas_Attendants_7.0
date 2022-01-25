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
public class TrajectoryAutoTest extends OpMode {

    public static boolean turn = false;
    public static boolean loop = false;
    public static double interval = 0;

    Robot robot = new Robot(hardwareMap);
    CornettCore motionProfile = new CornettCore(robot);

    @Override
    public void init() {

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Create Trajectories and Function Lists

        robot.intakeSys.regularFreightIntake();
        robot.lift.startServo();
    }

    @Override
    public void loop() throws InterruptedException {
        if (turn) {
            motionProfile.rotateSync(Math.toRadians(0), Math.toRadians(1));
        }

        else {
            motionProfile.runToPositionSync(24, 24, Math.toRadians(90), 1);
        }

        if(!loop) stop();
        sleep((long)interval);
    }
}
