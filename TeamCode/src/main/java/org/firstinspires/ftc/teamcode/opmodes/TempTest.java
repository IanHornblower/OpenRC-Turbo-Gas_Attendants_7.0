package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.util.Color;

import java.math.BigInteger;


@TeleOp
@Config
public class TempTest extends LinearOpMode {

    public static boolean DashTelemetryEnabled = true;
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;
    public static int BuildNumber = 23;
    public static String HubName = "Expansion Hub 1";
    public static boolean ImperialUnits = false;
    public static boolean StressTest = false;
    protected int stressNum = 1;
    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.clear();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);
        Battery.expansionHub = expansionHub;
        expansionHub.setLedColor(0,0,0);

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

            telemetry.clear();

            telemetry.addData("Loop Time", cycleLength+"ms");
            telemetry.addData("Time Running", (int)Math.floor((double)(timeRunning/1000))+" Seconds");
            telemetry.addData("Battery Voltage", Battery.voltage());
            telemetry.addData("Battery Current", Math.round(Battery.currentDraw())+"mA");
            telemetry.addData("Battery Apx. %", Battery.percentage()+"%");
            if (StressTest) telemetry.addData("Stress Test Number", stressNum);
            if(ImperialUnits)
                temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "°F";
            else temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.CELSIUS) + "°C";
            telemetry.addData("Temperature", temp);

            telemetry.update();

            if (StressTest) {
                if (stressNum < 1836311903) {
                    //stressNum = fib(stressNum);
                    stressNum = stressNum+stressNum;
                } else stressNum = 1;
            }



        }
    }

    static int fib(int n)
    {
        int a = 0, b = 1, c;
        if (n == 0)
            return a;
        for (int i = 2; i <= n; i++)
        {
            c = a + b;
            a = b;
            b = c;
        }
        return b;
    }

}
