package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
<<<<<<< HEAD
=======
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
>>>>>>> 214e7f6 (balls)
import org.firstinspires.ftc.teamcode.hardware.Controller;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.util.Color;

@Config
@TeleOp(name="Controller Test", group="Testing")
public class ControllerTest extends LinearOpMode {


    public static boolean DashTelemetryEnabled = true;
<<<<<<< HEAD
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;
    public static int BuildNumber = 18;
=======
    public double time = 0;
    public double cycleLength = 0;
    public static double timeRunning = 0;
<<<<<<< HEAD
    public static int BuildNumber = 235632752;
>>>>>>> 214e7f6 (balls)
=======
    public static String BuildNumber = "2.4.7";
    public static String BuildComment = "Updated Blinkin Timer";
>>>>>>> c7e8e04 (bbw pt2)
    public static String HubName = "Expansion Hub 1";
    public static boolean ImperialUnits = false;
    public static String timeUnit = "HOURS";
    public static boolean pain = false;
    ExpansionHubEx expansionHub;
    RevBlinkinLedDriver Driver;


    @Override
    public void runOpMode() throws InterruptedException {

<<<<<<< HEAD
        telemetry.clear();
=======
        if(pain) throw new OutOfMemoryError("feed me daddy");
>>>>>>> 214e7f6 (balls)
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);
        Driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        Blinkin.Driver = Driver;
        Battery.expansionHub = expansionHub;


        telemetry.addLine("Controller Test");
        telemetry.addData("Version", BuildNumber);
        telemetry.addLine("Ready");
        telemetry.update();


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

            gamepad1.update();

            telemetry.clear();


            // Swipe Testing Here






            if (gamepad1.right_trigger > 0)
                gamepad1.rumble((int)((2*cycleLength)/10*(gamepad1.right_trigger*10)));



            // Control Hub LED Testing
            if (gamepad1.circle) { expansionHub.setLedColor(255,0,0); Blinkin.setColor("red"); }
            else if (gamepad1.square) { expansionHub.setLedColor(255,0,255); Blinkin.setColor("hot pink"); }
            else if (gamepad1.cross) { expansionHub.setLedColor(0,0,255); Blinkin.setColor("blue"); }
            else if (gamepad1.triangle) { expansionHub.setLedColor(0,255,0);  Blinkin.setColor("green"); }
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
            temp = ImperialUnits ? expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "°F" : expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.CELSIUS) + "°C";
            telemetry.addData("Temperature", temp);

            Blinkin.updateLightTimer((int)timeRunning);

            telemetry.update();



        }
    }

}
