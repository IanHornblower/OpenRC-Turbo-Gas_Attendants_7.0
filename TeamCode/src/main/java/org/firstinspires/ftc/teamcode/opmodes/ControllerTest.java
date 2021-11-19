package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;

@TeleOp
@Config
public class ControllerTest extends LinearOpMode {

    public static boolean DashTelemetryEnabled = false;
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;
    public static int BuildNumber = 13;
    public static String HubName = "Expansion Hub 1";

    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);

        Battery.expansionHub = expansionHub;

        telemetry.addLine("Controller Test");
        telemetry.addData("Version", BuildNumber);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        time = System.currentTimeMillis();
        cycleLength = 0;
        timeRunning = 0;

        while(opModeIsActive()) {

            cycleLength = System.currentTimeMillis() - time;

            time = System.currentTimeMillis();
            timeRunning = timeRunning + cycleLength;

            Controller.updateTouchpad(gamepad1);

            telemetry.clear();

            telemetry.addData("1 Finger", gamepad1.touchpad_finger_1);
            telemetry.addData("2 Finger", gamepad1.touchpad_finger_2);
            telemetry.addData("1 Finger X", gamepad1.touchpad_finger_1_x);
            telemetry.addData("1 Finger Y", gamepad1.touchpad_finger_1_y);
            telemetry.addData("2 Finger X", gamepad1.touchpad_finger_2_x);
            telemetry.addData("2 Finger Y", gamepad1.touchpad_finger_2_y);
            telemetry.addData("Gamepad ID", gamepad1.getUser());
            telemetry.addData("Loop Time", cycleLength);
            telemetry.addData("Time Running", (int)Math.floor((double)(timeRunning/1000)));
            telemetry.addData("Battery Voltage", voltage());
            telemetry.addData("Battery Apx. %", Battery.percentage());

            if (gamepad1.right_bumper)
                gamepad1.rumble(100);

            if (gamepad1.options || gamepad2.options) {
               gamepad1.setGamepadId(2);
               //gamepad2.setGamepadId(1);
            }


            // Control Hub LED Testing
            if (gamepad1.cross) expansionHub.setLedColor(0,0,255);
            else if (gamepad1.circle) expansionHub.setLedColor(255,0,0);
            else if (gamepad1.square) expansionHub.setLedColor(255,175,175);
            else if (gamepad1.triangle) expansionHub.setLedColor(0,255,0);
            else expansionHub.setLedColor(255,255,255);

            telemetry.update();



        }
    }

    public double voltage() {
        // Returns infinity if we can't read the voltage
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor vSensor : hardwareMap.voltageSensor) {
            double voltage = vSensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


}
