package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.util.Color;

@Disabled
@Config
@TeleOp(name="Controller Test", group="Testing")
public class ControllerTest extends LinearOpMode {

    public static boolean DashTelemetryEnabled = true;
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;
    public static int BuildNumber = 18;
    public static String HubName = "Expansion Hub 1";
    public static boolean ImperialUnits = false;
    public static String timeUnit = "HOURS";
    public static boolean PhoneChargingEnabled = true;
    private double distance;
    private int csr;
    private int csg;
    private int csb;
    private DistanceUnit DU;
    ExpansionHubEx expansionHub;
    RevColorSensorV3 CSV3;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.clear();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);
        CSV3 = hardwareMap.get(RevColorSensorV3.class, "CSV3");
        Blinkin.Driver = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        Battery.expansionHub = expansionHub;
        DcMotor M = hardwareMap.get(DcMotor.class, "fl");
        if ( ImperialUnits ) DU = DistanceUnit.INCH;
        else DU = DistanceUnit.MM;

        telemetry.addLine("Controller Test");
        telemetry.addData("Version", BuildNumber);
        telemetry.addData("Charge Phone", PhoneChargingEnabled);
        telemetry.addLine("Ready");
        telemetry.update();

        expansionHub.setPhoneChargeEnabled(PhoneChargingEnabled);

        waitForStart();

        telemetry.clear();
        time = System.currentTimeMillis();
        cycleLength = 0;
        timeRunning = 0;
        String temp = "";

        while(opModeIsActive()) {

            cycleLength = System.currentTimeMillis() - time;

            time = System.currentTimeMillis();
            timeRunning = timeRunning + cycleLength;

            Controller.updateTouchpad(gamepad1);

            telemetry.clear();

            M.setPower(gamepad1.left_stick_y);

            if (gamepad1.right_bumper)
                gamepad1.rumble((int)cycleLength);

            distance = CSV3.getDistance(DU);
            csr = CSV3.red();
            csg = CSV3.green();
            csb = CSV3.blue();

            if ( ImperialUnits ) {
                if (distance < 0.5) Blinkin.setColor("white");
                else if (distance < 1.5) Blinkin.setColor("gray");
                else if (distance < 3) Blinkin.setColor("dark gray");
                else Blinkin.setColor("black");
            } else {
                if (distance < 25) Blinkin.setColor("white");
                else if (distance < 80) Blinkin.setColor("gray");
                else if (distance < 98) Blinkin.setColor("dark gray");
                else Blinkin.setColor("black");
            }

            // Control Hub LED Testing
            if (gamepad1.circle) expansionHub.setLedColor(255,0,0);
            else if (gamepad1.square) expansionHub.setLedColor(255,0,255);
            else if (gamepad1.cross) expansionHub.setLedColor(0,0,255);
            else if (gamepad1.triangle) expansionHub.setLedColor(0,255,0);
            else if (gamepad1.touchpad_finger_1) {
                double p1 = gamepad1.touchpad_finger_1_x;
                int p2 = (int)Math.round(((((100*p1)/2)+50)/100)*360);
                telemetry.addData("F1.X to Hue", p2);
                int[] pf = Color.HSVtoRGB((float)p2/360,1,1);
                telemetry.addData("R", pf[0]);
                telemetry.addData("G", pf[1]);
                telemetry.addData("B", pf[2]);
                expansionHub.setLedColor(pf[0], pf[1], pf[2]);
            } else expansionHub.setLedColor(255,255,255);


            telemetry.addData("Distance", distance);
            telemetry.addData("CS R", csr);
            telemetry.addData("CS G", csg);
            telemetry.addData("CS B", csb);
            telemetry.addData("1 Finger", gamepad1.touchpad_finger_1);
            telemetry.addData("2 Finger", gamepad1.touchpad_finger_2);
            telemetry.addData("1 Finger X", gamepad1.touchpad_finger_1_x);
            telemetry.addData("1 Finger Y", gamepad1.touchpad_finger_1_y);
            telemetry.addData("2 Finger X", gamepad1.touchpad_finger_2_x);
            telemetry.addData("2 Finger Y", gamepad1.touchpad_finger_2_y);
            telemetry.addData("Gamepad ID", gamepad1.getUser());
            telemetry.addData("Loop Time", cycleLength);
            telemetry.addData("Time Running", (int)Math.floor((double)(timeRunning/1000))+"ms");
            telemetry.addData("Battery Voltage", Battery.voltage());
            telemetry.addData("Battery Current", Math.round(Battery.currentDraw())+"mA");
            telemetry.addData("Battery Apx. %", Battery.percentage()+"%");
            telemetry.addData("Battery Time", Battery.timeRemaining(timeUnit));
            if(ImperialUnits)
                temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "°F";
            else temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.CELSIUS) + "°C";
            telemetry.addData("Temperature", temp);

            telemetry.update();



        }
    }

}
