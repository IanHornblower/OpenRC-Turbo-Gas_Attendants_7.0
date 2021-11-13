package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.controller.Swipe;
import org.firstinspires.ftc.teamcode.hardware.Battery;

import java.sql.Time;

@Config
@TeleOp(name = "Testing Controller", group = "Testing")
public class ControllerTest extends LinearOpMode {

    protected VoltageSensor vSensor =  hardwareMap.voltageSensor.get("Expansion Hub 1");

    public static boolean dashTelemeryEnabled = false;
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;


    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(dashTelemeryEnabled) telemetry = dashboard.getTelemetry();

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
            telemetry.addData("Battery Apx. %", Battery.percentage(vSensor));

            if (gamepad1.cross)
                gamepad1.rumble(100);

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
