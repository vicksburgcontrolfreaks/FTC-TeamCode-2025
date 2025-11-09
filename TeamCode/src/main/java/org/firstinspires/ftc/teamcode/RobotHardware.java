package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Central hardware wrapper — all motors, servos, sensors, LEDs
 */
public class RobotHardware {

    // === MOTORS ===
    public DcMotorEx lf, rf, lr, rr;
    public DcMotorEx shooter, collector, lWinch, rWinch;

    // === SERVOS ===
    public Servo flipper;

    // === SENSORS ===
    public VoltageSensor batteryVoltageSensor;

    // === VISION ===
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

    // === LED STRIP ===
    public RevBlinkinLedDriver leds;

    // === TELEMETRY ===
    public Telemetry telemetry;

    // === TIMERS ===
    private final ElapsedTime runtime = new ElapsedTime();

    // === CONFIG NAMES (CHANGE TO MATCH YOUR CONFIG) ===
    private static final String LF_NAME = "lf";
    private static final String RF_NAME = "rf";
    private static final String LR_NAME = "lr";
    private static final String RR_NAME = "rr";
    private static final String SHOOTER_NAME = "shooter";
    private static final String COLLECTOR_NAME = "collector";
    private static final String LWINCH_NAME = "lWinch";
    private static final String RWINCH_NAME = "rWinch";
    private static final String FLIPPER_NAME = "flipper";
    private static final String LED_NAME = "leds";

    /**
     * Initialize all hardware
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // === DRIVE MOTORS ===
        lf = hardwareMap.get(DcMotorEx.class, LF_NAME);
        rf = hardwareMap.get(DcMotorEx.class, RF_NAME);
        lr = hardwareMap.get(DcMotorEx.class, LR_NAME);
        rr = hardwareMap.get(DcMotorEx.class, RR_NAME);

//        lf.setDirection(DcMotor.Direction.REVERSE);
//        lr.setDirection(DcMotor.Direction.FORWARD);
//        rf.setDirection(DcMotor.Direction.FORWARD);
//        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === SHOOTER & COLLECTOR ===
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        collector = hardwareMap.get(DcMotorEx.class, COLLECTOR_NAME);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setTargetPositionTolerance(30);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === FLIPPER SERVO ===
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);
        flipper.setDirection(Servo.Direction.FORWARD);

        // === LIFT SYSTEM ===
        lWinch = hardwareMap.get(DcMotorEx.class, LWINCH_NAME);
        rWinch = hardwareMap.get(DcMotorEx.class, RWINCH_NAME);

        lWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        rWinch.setDirection(DcMotorSimple.Direction.FORWARD);

        lWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lWinch.setTargetPositionTolerance(30);
        lWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lWinch.setTargetPosition(0);
        lWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rWinch.setTargetPositionTolerance(30);
        rWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rWinch.setTargetPosition(0);
        rWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // === LED STRIP ===
//        try {
            leds = hardwareMap.get(RevBlinkinLedDriver.class, LED_NAME);
//            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
//            addTelemetry("LED", "Initialized");
//        } catch (Exception e) {
//            leds = null;
//            addTelemetry("LED", "NOT FOUND: " + e.getMessage());
//        }

        // === VISION (AprilTags) ===
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        // === VOLTAGE SENSOR ===
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        addTelemetry("Status", "Hardware Initialized");
    }

    /**
     * Safe telemetry add (won't crash if telemetry is null)
     */
    public void addTelemetry(String key, String format, Object... args) {
        if (telemetry != null) {
            telemetry.addData(key, format, args);
        }
    }

    public void addTelemetry(String key, String value) {
        if (telemetry != null) {
            telemetry.addData(key, value);
        }
    }

    /**
     * Get battery voltage
     */
    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    /**
     * Reset runtime timer
     */
    public void resetRuntime() {
        runtime.reset();
    }

    public double getRuntime() {
        return runtime.seconds();
    }
}