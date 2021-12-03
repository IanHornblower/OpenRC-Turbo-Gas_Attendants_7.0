package org.firstinspires.ftc.teamcode.revextensions2;

import java.util.HashMap;
import java.util.Map;

public class TemperatureMap {

    private Map <Integer, Integer> TempMap = null;

    /**
     * Setups temperature map
     */
    private void SetupTempMap() {
        /*
         *
         *
         *
         *
         */
    }

    public double adjustedHubTemperature(int temp, ExpansionHubEx.TemperatureUnits unit) {
        if(TempMap == null) SetupTempMap();

        return temp;
    }

}
