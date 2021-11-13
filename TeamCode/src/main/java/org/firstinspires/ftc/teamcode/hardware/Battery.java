package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public abstract class Battery {

    public static double percentage(VoltageSensor vSensor) {
        // Returns 0 if Battery.voltage() returns ∞
        double result = 0;
        double v = vSensor.getVoltage();
        if(v != Double.POSITIVE_INFINITY) {
            result = (v-10)*20;
            if(result < 0)
                result = 0;
        }
        return result;
    }

}
