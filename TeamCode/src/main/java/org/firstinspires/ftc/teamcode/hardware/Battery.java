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
        int p = percentage();
        double a = currentDraw();
        double ret;
        if ( a < 1 ) return -1;
        ret = Math.round(((float)p/100)*60*(BatteryCapacity/a));
        if (Objects.equals(timeUnit, "HOURS")) {
            ret = Math.round(ret/60);
        } else if (Objects.equals(timeUnit, "SECONDS")) {
            ret = Math.round(ret*60);
        }
        return (int)Math.round(ret);
    }

    public static double currentDraw() {
        if (expansionHub == null) throw new NullPointerException("Battery.expansionHub is Not Defined");
        return expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
    }

    public static double voltage() {
        if (expansionHub == null) throw new NullPointerException("Battery.expansionHub is Not Defined");
        return Math.round(expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS));
    }


}
