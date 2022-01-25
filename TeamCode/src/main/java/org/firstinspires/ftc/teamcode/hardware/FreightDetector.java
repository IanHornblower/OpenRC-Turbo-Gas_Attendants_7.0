package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class FreightDetector {

    public static double threshold = 0;
    private volatile boolean isRunning = true;

    Robot robot;
    ColorSensor freightDetector_color;
    DistanceSensor freightDetector_distance;

    public FreightDetector(Robot robot) {
        this.robot = robot;
        freightDetector_color = robot.getFreightSensor_color();
        freightDetector_distance = robot.getFreightSensor_distance();
    }

    public double distance() {
        return freightDetector_distance.getDistance(DistanceUnit.MM);
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
