package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Autonomous(name = "DashboardTest", group = "Testing")
public class DashboardTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        double x = 0, y = 0, h = 0;

        waitForStart();

        telemetry.clear();
        telemetry.addLine("Active");
        telemetry.update();

        while(opModeIsActive()) {
            camera.startCameraStream(0);


            TelemetryPacket packet = new TelemetryPacket();

            x -= gamepad1.left_stick_y/3000;
            y += gamepad1.left_stick_x/3000;

            h -= Math.toRadians(gamepad1.right_stick_x/1000);

            telemetry.addData("Location", camera.sDeterminePosition());

            //Field field = new Field(packet);

            //field.createCircularRobot(new Pose2D(x, y, h));


            //dashboard.sendTelemetryPacket(field.getPacket());
        }
    }
}