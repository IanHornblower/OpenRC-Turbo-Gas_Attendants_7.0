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

@Autonomous(name = "All Auto(s)", group = "Auto")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        robot.telemetry.addLine("Starting Hardware Map & Camera");
        robot.telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        // Create Trajectories

        robot.telemetry.clear();
        robot.telemetry.addLine("Waiting For Start");
        robot.telemetry.addLine(
                "Autonomous Configuration: \n" +
                         "Side: RED");
        robot.telemetry.addData("Location", camera.sDeterminePosition());
        robot.telemetry.update();

        waitForStart();

        robot.telemetry.clear();
        robot.telemetry.update();

        while(opModeIsActive()) {

        switch(AutoConfig.side) {
            case RED:
                /*
                 * Init
                 */

                robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

                /*
                 * Run Auto
                 */

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
                break;

            case BLUE:
                switch(AutoConfig.park) {
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
