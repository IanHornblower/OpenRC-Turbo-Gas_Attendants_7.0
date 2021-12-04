package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Map;

public class Controller extends Gamepad {

    public void update() {


        if (!this.touchpad_finger_1) {
            this.touchpad_finger_1_x = 0;
            this.touchpad_finger_1_y = 0;
        }

        if (!this.touchpad_finger_2) {
            this.touchpad_finger_2_x = 0;
            this.touchpad_finger_2_y = 0;
        }

    }


    private Double[] Loc1 = {0d, 0d};
    private Double[] Loc2 = {0d, 0d};
    private Double[] Loc3 = {0d, 0d};


}
