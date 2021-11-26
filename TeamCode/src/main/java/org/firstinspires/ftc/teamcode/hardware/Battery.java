package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;

import java.util.Objects;

public class Battery {

    public static ExpansionHubEx expansionHub;
    public static int BatteryCapacity = 3000;

    public static int percentage() {
        int result = 0;
        double v = Math.round(voltage()/10)/100;
        result = (int)(v-10)*20;
        if(result < 0)
            result = 0;
        return Math.round(result);
    }

    public static int timeRemaining(String timeUnit) {
        float p = (float)percentage();
        float a = (float)currentDraw();
        double ret = Math.round((p/100)*60*(BatteryCapacity/a));
        if (Objects.equals(timeUnit, "HOURS")) {
            ret = Math.round(ret/60);
        } else if (Objects.equals(timeUnit, "SECONDS")) {
            ret = Math.round(ret*60);
        }
        return (int)Math.round(ret);
    }

    public static double currentDraw() {
        return expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
    }

    public static double voltage() {
        return Math.round(expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS));
    }


}
