package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {

    public static void updateTouchpad(Gamepad gamepad) {

        if (!gamepad.touchpad_finger_1) {
            gamepad.touchpad_finger_1_x = 0;
            gamepad.touchpad_finger_1_y = 0;
        }

        if (!gamepad.touchpad_finger_2) {
            gamepad.touchpad_finger_2_x = 0;
            gamepad.touchpad_finger_2_y = 0;
        }

    }

    public static void changeDriver(Gamepad gamepad) {



    }

}
