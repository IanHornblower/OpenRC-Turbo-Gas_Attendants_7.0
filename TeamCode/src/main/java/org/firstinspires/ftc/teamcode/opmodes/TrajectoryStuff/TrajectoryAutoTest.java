package org.firstinspires.ftc.teamcode.opmodes.TrajectoryStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

@Autonomous(name = "Trajectory Testing", group = "Traj")
public class TrajectoryAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);

        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

        // Create Trajectories and Function Lists

        waitForStart();

        while(opModeIsActive()) {

            stop();
        }
    }
}
