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

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Create Trajectories and Function Lists

        Trajectory joe  = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(0, 24));
        joe.addWaypoint(new Point(24, 36));
        joe.addWaypoint(new Point(48, 48));

        ArrayList<Function> list = new ArrayList<>();

        list.add(new Function(12, () -> {
            telemetry.addLine("AT 12: \t" + robot.accumulatedDistance);
            telemetry.update();
        }));

        list.add(new Function(24,() -> {
            telemetry.addLine("do");
            telemetry.update();
        }));

        list.add(new Function(36, () -> {
            telemetry.addLine("AT 36: \t" + robot.accumulatedDistance);
            telemetry.update();
        }));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();
            
            joe.at(list).followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 8, 1);

            motionProfile.runToPositionSync(0,0, 0, 1);

            sleep(10000);

            //stop();
        }
    }
}
