package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Map;

public class Controller extends Gamepad {

    public void update() {
<<<<<<< HEAD


        if (!this.touchpad_finger_1) {
            this.touchpad_finger_1_x = 0;
            this.touchpad_finger_1_y = 0;
=======
        updateInputs();
        cycleLength = System.currentTimeMillis() - time;
        time = System.currentTimeMillis();
        ttime = ttime + cycleLength;

        // Set Touch Positions to 0 when you let go
        if (!gamepad.touchpad_finger_1) {
            gamepad.touchpad_finger_1_x = 0;
            gamepad.touchpad_finger_1_y = 0;
        }
        if (!gamepad.touchpad_finger_2) {
            gamepad.touchpad_finger_2_x = 0;
            gamepad.touchpad_finger_2_y = 0;
        }

        // Set Touch Region Bools
        touchingTop = gamepad.touchpad_finger_1_y > 0;
        touchingBottom = gamepad.touchpad_finger_1_y < 0;
        touchingLeft = gamepad.touchpad_finger_1_x > 0;
        touchingRight = gamepad.touchpad_finger_1_x < 0;
        touchingTopLeft = touchingTop && touchingLeft;
        touchingTopRight = touchingTop && touchingRight;
        touchingBottomLeft = touchingBottom && touchingLeft;
        touchingBottomRight = touchingBottom && touchingRight;

        Log.i("Controller.update()", "Controller Updated");
        Log.i("Controller.update()", Boolean.toString(a));
        Log.i("Controller.update()", Boolean.toString(b));
        Log.i("Controller.update()", Float.toString(left_stick_x));
        Log.i("Controller.update()", Float.toString(left_stick_y));


        if(gamepad.touchpad_finger_1 && ttime > 250) {
            loc3 = new Float[]{loc2[0], loc2[1]};
            loc2 = new Float[]{loc1[0], loc1[1]};
            loc1 = new Float[]{gamepad.touchpad_finger_1_x, gamepad.touchpad_finger_1_y};
            ttime = 0;
>>>>>>> 214e7f6 (balls)
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
