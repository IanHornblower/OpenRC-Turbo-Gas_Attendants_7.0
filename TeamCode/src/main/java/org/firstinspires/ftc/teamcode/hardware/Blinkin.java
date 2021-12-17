package org.firstinspires.ftc.teamcode.hardware;


import android.util.Log;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import static com.sun.tools.javac.util.StringUtils.toLowerCase;

public class Blinkin {

    public static RevBlinkinLedDriver Driver = null;

    public static void setColor(String cname) {

        if (Driver == null) throw new NullPointerException("Blinkin.Driver is Not Defined");

        switch (toLowerCase(cname)) {
            case "dark gray":  Driver.setPattern(BlinkinPattern.DARK_GRAY); break;
            case "gray":       Driver.setPattern(BlinkinPattern.GRAY); break;
            case "white":      Driver.setPattern(BlinkinPattern.WHITE); break;
            case "violet":     Driver.setPattern(BlinkinPattern.VIOLET); break;
            case "blue":       Driver.setPattern(BlinkinPattern.BLUE); break;
            case "aqua":       Driver.setPattern(BlinkinPattern.AQUA); break;
            case "green":      Driver.setPattern(BlinkinPattern.GREEN); break;
            case "lime":       Driver.setPattern(BlinkinPattern.LIME); break;
            case "yellow":     Driver.setPattern(BlinkinPattern.YELLOW); break;
            case "orange":     Driver.setPattern(BlinkinPattern.ORANGE); break;
            case "red":        Driver.setPattern(BlinkinPattern.RED); break;
            case "hot pink":   Driver.setPattern(BlinkinPattern.HOT_PINK); break;
            default:           Driver.setPattern(BlinkinPattern.BLACK); break;
        }

    }

    public static synchronized void updateLightTimer(int currentTime) {
        updateLightTimer(currentTime, false);
    }

    /**
     * The 'auto' boolean is optional, just set true for auto
     * @param currentTime time in lmao
     * @param auto is it auto? not required lmao
     */
    public static synchronized void updateLightTimer(int currentTime, boolean auto) {
        int matchLength = auto ? 30 : 150;
        currentTime = matchLength-(currentTime/1000);
        Log.i("Blinkin Time", String.valueOf(currentTime));
        switch(currentTime) {
            case(5):
            case(3):
            case(1):
                Driver.setPattern(BlinkinPattern.WHITE);
                break;
            case(4):
            case(2):
            case(0):
                Driver.setPattern(BlinkinPattern.RED);
                break;

            default:
                Driver.setPattern(BlinkinPattern.BLACK);
                break;
        }



    }



}
