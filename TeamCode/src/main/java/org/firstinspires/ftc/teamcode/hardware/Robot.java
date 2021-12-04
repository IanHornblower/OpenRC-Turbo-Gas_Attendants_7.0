package org.firstinspires.ftc.teamcode.hardware;

import android.os.health.ServiceHealthStats;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

public class Robot extends OpMode {
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    private DcMotor leftEncoder, rightEncoder, lateralEncoder;

    private DcMotor liftMotor;
    private CRServo boxServo;

    private DcMotor intake;
    private CRServo intakeServo;

    DcMotor left;
    DcMotor right;

    private BNO055IMU imu;
    private Orientation angles;

    private double previousHeading;

    public HardwareMap hwMap;
    public DriveTrain DriveTrain;
    public IMU IMU;
    public org.firstinspires.ftc.teamcode.hardware.lift lift;
    public intake intakeSys;
    public spinMotor spinMotor;

    public enum controlType{ROBOT, FIELD}

    // Robot Kinematics

    // Odmometric Constraints
    public final static double L = 11;  // separation between left and right Encoder.
    public final static double lateralOffset = -6.33;  // offset between origin of robot and lateral Encoder.
    public final static double R = 0.688975;  // Encoder wheel radius.
    public final static double encoderTicksPerRev = 8192;  // Ticks read per revolution of REV Encoder.
    public final static double inchPerTick = 2.0 * Math.PI * R / encoderTicksPerRev;  // Inches traveled per tick moved.


    // Velocity

    int cycleToSkip = 20;

    public Robot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        frontRight = hardwareMap.dcMotor.get("fr");
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hardwareMap.dcMotor.get("bl");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hardwareMap.dcMotor.get("br");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boxServo = hardwareMap.crservo.get("boxServo");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = hardwareMap.crservo.get("intakeServo");

        left = hardwareMap.dcMotor.get("duckLeft");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right = hardwareMap.dcMotor.get("duckRight");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder = frontLeft;
        //lateralEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = backRight;
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = backLeft;
        //leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        stopDrive();
        resetDriveEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DriveTrain = new DriveTrain(this);
        IMU = new IMU(imu);
        lift = new lift(this);
        intakeSys = new intake(this);
        spinMotor = new spinMotor(this);
    }

    public DcMotor getLift() {
        return liftMotor;
    }

    public CRServo getBoxServo() {
        return boxServo;
    }

    public CRServo getIntakeServo() {
        return intakeServo;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getLeft() {
        return left;
    }

    public DcMotor getRight() {
        return right;
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getBackLeft() {
        return backLeft;
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getBackRight() {
        return backRight;
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getFrontEncoder() {
        return lateralEncoder;
    }


    private void resetDriveEncoders() {
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // update variables
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentLateralPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldLateralPosition = 0;

    public double accumulatedHeading = 0;

    public Pose2D START_POSITION = new Pose2D(0, 0, AngleUtil.interpretAngle(90));  // Default

    public void setSTART_POSITION(Pose2D START) {
        START_POSITION = START;
        pos = START;
    }

    public Pose2D pos = START_POSITION;

    public void updateAcumulatedHeading() {
        double currentHeading = Math.toDegrees(pos.getHeading());

        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading -= dHeading;
        previousHeading = currentHeading;
    }

    public void updateOdometry() { // make update() --> odometry() if need be [Merge with UpdateVelocities()
        currentRightPosition = -rightEncoder.getCurrentPosition(); // Invert in Necessary
        currentLeftPosition = leftEncoder.getCurrentPosition(); // Invert in Necessary
        currentLateralPosition = lateralEncoder.getCurrentPosition(); // Invert in Necessary

        int dnRight = currentRightPosition - oldRightPosition;
        int dnLeft = currentLeftPosition - oldLeftPosition;
        int dnLateral = currentLateralPosition - oldLateralPosition;

        double dtheta = (dnLeft - dnRight) / L;
        double dx = (dnLeft+dnRight) / 2.0;
        double dy = dnLateral - lateralOffset * dtheta;

        dtheta *= inchPerTick;
        dx *= inchPerTick;
        dy *= inchPerTick;

        //double theta = pos.heading + (dtheta / 2.0);  // Does same thing as pos.heading | Might remove
        double dxTraveled = dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        double dyTraveled = dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);

        pos.x -= dxTraveled;  // Inverted cuz it was negative? :)
        pos.y += dyTraveled;
        pos.heading += dtheta;

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldLateralPosition = currentLateralPosition;

        telemetry.addData("XYH", pos.toString());
        //telemetry.update();
    }

    double oldX = 0;
    double oldY = 0;
    double oldH = 0;
    double oldTime = 0;

    double currentX = 0;
    double currentY = 0;
    double currentH = 0;
    double currentTime = 0;

    public double dx = 0;
    public double dy = 0;
    public double dt = 0;
    public double dTheta = 0;
    int i = 0;

    public void updateVelocity() { // make update() --> odometry() if need be [Merge With UpdateOdometry()]
        i++; if(i % cycleToSkip == 0) {
                oldX = currentX;
                oldY = currentY;
                oldH = currentH;
                oldTime = currentTime;

                currentX = pos.x;
                currentY = pos.y;
                currentH = pos.getHeading();
                currentTime = System.nanoTime()/1e+9;

                dx = currentX - oldX;
                dy = currentY - oldY;
                dTheta = currentH - oldH;
                dt = currentTime - oldTime;

                pos.xVelocity = dx/dt;
                pos.yVelocity = dy/dt;
                pos.headingVelocity = dTheta/dt;
        }
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}