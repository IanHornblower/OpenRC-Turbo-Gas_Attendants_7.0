package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;

import static org.firstinspires.ftc.teamcode.util.Controller.LEFT_TRIGGER_X_POW;
import static org.firstinspires.ftc.teamcode.util.Controller.LEFT_TRIGGER_Y_POW;

@Disabled
@Config
@TeleOp(name = "RedSide TeleOp", group = "Comp")
public class DemoLmoa extends LinearOpMode {

    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        Battery.expansionHub = expansionHub;
        robot.setSTART_POSITION(PoseStorage.autoEnd);


        waitForStart();

        while(opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            robot.updateOdometry();

            // Drive Train

            double Ch3 = Controller.deadZone(-gamepad1.left_stick_x, 0.1);
            double Ch4 = Controller.deadZone(-gamepad1.left_stick_y, 0.1);
            double Ch1 = Controller.deadZone(-gamepad1.right_stick_x, 0.1);


            robot.getFrontLeft().setPower(Ch3+Ch1-Ch4);
            robot.getBackLeft().setPower(-Ch3+Ch1-Ch4);
            robot.getFrontRight().setPower(Ch3+Ch1+Ch4);
            robot.getBackRight().setPower(-Ch3+Ch1+Ch4);

            // Duck Motor

            robot.spinMotor.run(gamepad2.right_bumper);
            robot.spinMotor.reverse(gamepad2.left_bumper);

            // Lift System

            //if(gamepad2.dpad_down) {
            //    robot.lift.setServoState(lift.SERVOSTATE.ONE);
            //}
//
            //if(gamepad2.dpad_right) {
            //    robot.lift.setServoState(lift.SERVOSTATE.TWO);
            //}
//
            //if(gamepad2.dpad_up) {
            //    robot.lift.setServoState(lift.SERVOSTATE.THREE);
            //}
//
            //if(gamepad2.dpad_left) {
            //    robot.lift.setServoState(lift.SERVOSTATE.INTAKE_LEVEL);
            //}

            if(gamepad2.y) {
                robot.lift.primeLift();
            }

            //if(gamepad2.b) {
            //    robot.lift.scoreFreight();
            //}

            if(gamepad2.a) {
                robot.lift.returnLift();
            }

            //if(gamepad1.dpad_right) {
            //    robot.intakeSys.duckIntake();
            //}

            robot.getLift().setPower(gamepad2.left_stick_y);

            robot.intakeSys.reverse(gamepad2.left_trigger > 0.1);

            robot.intakeSys.run(gamepad2.right_trigger > 0.1);

            telemetry.addData("Lift", robot.lift.state.toString());
            telemetry.addData("Battery Draw", Battery.currentDraw());
            telemetry.addData("Lift Motor Enc", robot.getLift().getCurrentPosition());

            telemetry.addData("\nXYH", robot.pos.toString());
            telemetry.update();
        }
    }
}