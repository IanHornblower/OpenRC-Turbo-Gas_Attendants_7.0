package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Array;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@Autonomous(name = "RedSide Auto", group = "Auto")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);

        CameraStreamServer.getInstance().setSource(camera.getCamera());

        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

        // Create Trajectories

        telemetry.clear();
        telemetry.addLine("Waiting For Start");
        telemetry.addLine(
                "Autonomous Configuration: \n" +
                         "Side: RED");
        telemetry.addData("Location", camera.sDeterminePosition());
        telemetry.update();

        CornettCore motionProfile = new CornettCore(robot);

        waitForStart();

        telemetry.clear();
        telemetry.update();

        while(opModeIsActive()) {

            motionProfile.runToPositionSync(39, -29, Math.toRadians(315),1);
            robot.DriveTrain.stopDrive();
            sleep(2000);

            motionProfile.runToPositionSync(52, -62, Math.toRadians(0), 1);
            robot.DriveTrain.stopDrive();

            robot.DriveTrain.setMotorPowers(-0.3, -0.2);
            sleep(400);
            robot.DriveTrain.stopDrive();

            sleep(500);
            robot.getDuck().setPower(-0.5);
            sleep(3000);
            robot.getDuck().setPower(0.0);

            motionProfile.runToPositionSync(38, -59, Math.toRadians(90), 1);
            robot.DriveTrain.stopDrive();

            PoseStorage.autoEnd = robot.pos;
            camera.shutdownPipeline();

            stop();
        }
    }
}
