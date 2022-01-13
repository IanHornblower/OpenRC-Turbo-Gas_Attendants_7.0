package org.firstinspires.ftc.teamcode.opmodes.Comp;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.*;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;
import static org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera.position.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.system.RefCounted;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@Autonomous(name = "All Auto(s)", group = "Comp")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        lift.LIFT pos = D1;

        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        while(!isStarted()) {
            telemetry.clear();
            telemetry.addLine("Waiting For Start");
            telemetry.addLine(
                    "Autonomous Configuration: \n" +
                            "Side: " + MatchConfig.side.toString() +
                            "\nParking Location: " + MatchConfig.park.toString());
            telemetry.addData("Location", camera.sDeterminePosition());
            telemetry.update();
        }

        telemetry.clear();
        telemetry.update();

        switch (camera.determinePosition()) {
            case A:
                pos = D1;
            case B:
                pos = D2;
            case C:
                pos = D3;
            default:
                pos = D1;
        }

        while(opModeIsActive()) {

        switch(MatchConfig.side) {
            case RED:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        /*
                         * Init
                         */

                        robot.setSTART_POSITION(new Pose2D(63, 13, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                         */

                        robot.lift.prime(pos);

                        motionProfile.runToPositionSync(36, 5, Math.toRadians(45), 1);

                        robot.lift.drop();

                        sleep(200);

                        robot.lift.retract();

                        Trajectory toWarehouse = new Trajectory(robot, new Pose2D(36, 5, Math.toRadians(45)));

                        toWarehouse.addWaypoint(new Point(60.0, 12.0));
                        toWarehouse.addWaypoint(new Point(72, 30));
                        toWarehouse.addWaypoint(new Point(72, 45));

                        toWarehouse.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);

                        break;
                    case STORAGE:
                        /*
                         * Init
                         */

                        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                         */

                        robot.lift.prime(pos);

                        motionProfile.runToPositionSync(39, -29, Math.toRadians(315),1);
                        robot.DriveTrain.stopDrive();

                        robot.lift.drop();

                        sleep(200);

                        robot.lift.retract();

                        motionProfile.runToPositionSync(50, -62, Math.toRadians(0), 1);
                        robot.DriveTrain.stopDrive();

                        robot.DriveTrain.setMotorPowers(-0.2, -0.3);
                        sleep(400);
                        robot.DriveTrain.stopDrive();

                        sleep(500);
                        robot.getDuck().setPower(-0.5);
                        sleep(3000);
                        robot.getDuck().setPower(0.0);

                        motionProfile.runToPositionSync(34, -59, Math.toRadians(0), 1);
                        robot.DriveTrain.stopDrive();
                        break;
                }
                break;
            case BLUE:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        break;
                    case STORAGE:
                        break;
                }
                break;
        }


            PoseStorage.autoEnd = robot.pos;
            camera.shutdownPipeline();

            stop();
        }
    }
}
