package org.firstinspires.ftc.teamcode.opmodes.Comp;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.*;
import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;
import static org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera.position.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.system.RefCounted;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Function;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.ArrayList;

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

        robot.lift.primeServo();

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

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(40, 7, AngleUtil.interpretAngle(0), 1);
                        robot.DriveTrain.stopDrive();

                        motionProfile.rotateSync(Math.toRadians(45), Math.toRadians(5));

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        // Wait 5 Seconds -> for testing not running

                        sleep(5000);

                        boolean runTo = true;

                        if(runTo) {
                            motionProfile.runTimeSync(0.3, 300);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();

                        await(500, ()-> {
                            robot.lift.startServo();
                            robot.lift.setPosition(lift.liftStart);
                        });

                        boolean split1 = false;

                        if(split1) {
                            // Tune
                            motionProfile.runToPositionSync(60, 7, Math.toRadians(45), 1);
                            robot.DriveTrain.stopDrive();

                            motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(5));
                        }
                        else {
                            // Tune
                            motionProfile.runToPositionSync(60, 7, Math.toRadians(90), 3);
                        }


                        robot.DriveTrain.setMotorPowers(1, 0, 0);
                        sleep(1500);
                        robot.stopDrive();

                        robot.DriveTrain.setMotorPowers(0, 1, 0);
                        sleep(1500);
                        robot.stopDrive();

                        stop();

                        break;
                    case STORAGE:
                        /*
                         * Init
                         */
                        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                         */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        // Tune

                        motionProfile.runToPositionSync(38, -28, AngleUtil.interpretAngle(0),1);
                        robot.DriveTrain.stopDrive();

                        motionProfile.rotateSync(AngleUtil.interpretAngle(300), Math.toRadians(5));

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        // Wait 5 Seconds -> for testing not running

                        sleep(5000);

                        runTo = true;

                        if(runTo) {
                            motionProfile.runTimeSync(0.3, 300);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1200);

                        robot.lift.primeServo();
                        await(500, ()-> {
                            robot.lift.startServo();
                            robot.lift.setPosition(lift.liftStart);
                        });

                        // Tune

                        robot.DriveTrain.setMotorPowers(-0.7, -0.7); //may need to move more to the left or right
                        sleep(1200);   // Tune so it hits the duck wheel
                        robot.DriveTrain.stopDrive();

                        robot.DriveTrain.setMotorPowers(-0.3, -0.3); // Maybe remove

                        sleep(500);
                        robot.DriveTrain.stopDrive();

                        robot.getDuck().setPower(-0.5);
                        sleep(3000);
                        robot.getDuck().setPower(0.0);

                        robot.intakeSys.raiseIntake();

                        // Tune
                        motionProfile.runToPositionSync(36, -59, AngleUtil.interpretAngle(300), 1);
                        robot.DriveTrain.stopDrive();

                        motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(5));

                        stop();
                        break;
                }
                break;
            case BLUE:
                switch(MatchConfig.park) {
                    case WAREHOUSE:

                    case STORAGE:
                        
                }
                break;
        }


            PoseStorage.autoEnd = robot.pos;
            camera.shutdownPipeline();

            stop();
        }
    }
}
