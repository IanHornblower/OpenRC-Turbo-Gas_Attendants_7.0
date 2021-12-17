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

        //CameraStreamServer.getInstance().setSource(camera.getCamera());

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(0)));  // Change

        // Create Trajectories

        telemetry.clear();
        telemetry.addLine("Waiting For Start");
        telemetry.addLine(
                "Autonomous Configuration: \n" +
                         "Side: RED");
        telemetry.addData("Location", camera.sDeterminePosition());
        telemetry.update();

        waitForStart();

        telemetry.clear();
        telemetry.update();

        while(opModeIsActive()) {
           ////robot.updateOdometry();

           ////CornettCore motionProfile = new CornettCore(robot);

            switch(AutoConfig.side) {
                case RED:
                    switch(AutoConfig.type) {
                        case DUCK:

                            switch(camera.determinePosition()) {
                                case A:
                                    camera.shutdownPipeline();



                                    break;

                                case B:

                                case C:

                                default:
                            }


                            break;

                        case WAREHOUSE:

                            switch(camera.determinePosition()) {
                                case A:

                                    break;

                                case B:

                                case C:

                                default:
                            }

                            break;

                        default:

                            break;
                    }
                    break;
                case BLUE:
                    // Do
                    switch(AutoConfig.type) {
                        case DUCK:

                            switch(camera.determinePosition()) {
                                case A:




                                    break;

                                case B:
                                    camera.shutdownPipeline();

                                case C:

                                default:
                            }


                            break;

                        case WAREHOUSE:

                            switch(camera.determinePosition()) {
                                case A:

                                    break;

                                case B:

                                case C:

                                default:
                            }

                            break;

                        default:

                            break;
                    }
                    break;
            }

            switch(AutoConfig.type) {
                case DUCK:

                    switch(camera.determinePosition()) {
                        case A:
                            camera.shutdownPipeline();



                            break;

                        case B:

                        case C:

                        default:
                    }


                    break;

                case WAREHOUSE:

                    switch(camera.determinePosition()) {
                        case A:

                            break;

                        case B:

                        case C:

                        default:
                    }

                    break;

                default:

                    break;
            }

            PoseStorage.autoEnd = robot.pos;

            //stop();
        }
    }
}
