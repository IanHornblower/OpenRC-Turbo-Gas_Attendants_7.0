package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

public class Blinkin {

    public static RevBlinkinLedDriver Driver = null;

    public static void setColor(String cname) {

        if (Driver == null) throw new NullPointerException("Blinkin.Driver is Not Defined");

        switch (cname) {
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



}
