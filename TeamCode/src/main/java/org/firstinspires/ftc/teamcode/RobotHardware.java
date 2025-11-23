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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
    private static final double CAMERA_FOV_DEG = 78.0;  // Anker C200 default

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

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === SHOOTER & COLLECTOR ===
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        collector = hardwareMap.get(DcMotorEx.class, COLLECTOR_NAME);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setTargetPositionTolerance(30);

        // Configure shooter PIDF for velocity control
        // P: Proportional gain (tune for responsiveness)
        // I: Integral gain (usually 0 for flywheel)
        // D: Derivative gain (usually 0 for flywheel)
        // F: Feed-forward (based on motor max velocity and battery voltage)
        double batteryVoltage = getBatteryVoltage();
        double maxVelocity = 2800.0;  // Max ticks/sec at 12V - tune this based on your motor
        double F = (32767.0 / maxVelocity) * (12.0 / batteryVoltage);
        shooter.setVelocityPIDFCoefficients(1.5, 0.0, 0.0, F);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === FLIPPER SERVO ===
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);
        flipper.setDirection(Servo.Direction.FORWARD);
        // Don't set position during init - will be set in start()

        // === LIFT SYSTEM ===
        lWinch = hardwareMap.get(DcMotorEx.class, LWINCH_NAME);
        rWinch = hardwareMap.get(DcMotorEx.class, RWINCH_NAME);

        lWinch.setDirection(DcMotorSimple.Direction.FORWARD);
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
        // Anker C200: 78° FOV → fx = (width/2) / tan(FOV/2)
        double fx = (1280 / 2) / Math.tan(Math.toRadians(CAMERA_FOV_DEG / 2));
        double fy = fx;  // assume square pixels
        double cx = 1280 / 2;
        double cy = 720 / 2;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(fx, fy, cx, cy)
                .setCameraPose(
                        new Position(DistanceUnit.INCH,
                                2.0,   // X: camera 2" right of center
                                -7.0,   // Y: camera 7" back from center
                                6.0,   // Z: camera 6" above ground
                                0),
                        new YawPitchRollAngles(AngleUnit.DEGREES,
                                180,    // YAW: back-facing
                                20,    // PITCH: horizontal
                                0,      // ROLL: level
                                0)
                )
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Change name if needed
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(1280, 720))  // 720p
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

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