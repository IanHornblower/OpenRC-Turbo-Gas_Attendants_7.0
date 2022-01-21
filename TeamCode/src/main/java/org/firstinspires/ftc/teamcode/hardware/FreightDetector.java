package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FreightDetector {

    public static double threshold = 0;
    private volatile boolean isRunning = true;

    Robot robot;
    ColorRangeSensor freightDetector;

    public FreightDetector(Robot robot) {
        this.robot = robot;
        freightDetector = robot.getFreightSensor();
    }

    public double distance() {
        return freightDetector.getDistance(DistanceUnit.MM);
    }

    public boolean hasFreight() {
        return distance() < threshold;
    }

    public void run() {
        Thread thread = new Thread(()-> {
            boolean isGreen = false;

            while(isRunning) {
                isGreen = hasFreight();

                if(hasFreight() && !isGreen) {
                    Blinkin.setColor("green");
                }
                else {
                    Blinkin.setColor("red");
                }
            }
        });
    }

    public void close() {
        isRunning = false;
    }
}
