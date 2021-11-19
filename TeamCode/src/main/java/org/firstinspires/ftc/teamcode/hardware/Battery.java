package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;

public class Battery {

    public static ExpansionHubEx expansionHub;


    public static double percentage() {
        // Returns 0 if Battery.voltage() returns ∞
        double result = 0;
        double v = expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        if(v != Double.POSITIVE_INFINITY) {
            result = (v-10)*20;
            if(result < 0)
                result = 0;
        }
        return result;
    }

}
