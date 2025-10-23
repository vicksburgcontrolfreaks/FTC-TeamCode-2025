package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotHardware {
    // Drive motors
    public DcMotorEx rf, rr, lr, lf;
    // Mechanisms
    public DcMotorEx collector;
    public DcMotorEx shooter;
    public Servo flipper;
    // Sensors
    public DigitalChannel sensor1, sensor2, sensor3;
    // Vision
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;
    // Telemetry
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry; // Initialize telemetry from parameter

        // Drive motors
        rf = hardwareMap.get(DcMotorEx.class, "rf"); // EH1
        rr = hardwareMap.get(DcMotorEx.class, "rr"); // EH0
        lr = hardwareMap.get(DcMotorEx.class, "lr"); // EH3
        lf = hardwareMap.get(DcMotorEx.class, "lf"); // EH2

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Mechanisms
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flipper = hardwareMap.get(Servo.class, "flipper");
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure motors
        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Sensors
        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensor3 = hardwareMap.get(DigitalChannel.class, "sensor3");
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);
        sensor3.setMode(DigitalChannel.Mode.INPUT);

        // AprilTag vision
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        telemetry.addData("Hardware Status", "Initialized");
        telemetry.update();
    }

    public boolean isMagazineFull() {
        boolean full = !sensor1.getState() && !sensor2.getState() && !sensor3.getState();
        telemetry.addData("Magazine Full", full);
        telemetry.update();
        return full;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void addTelemetry(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public void addTelemetry(String caption, double value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public void addTelemetry(String caption, boolean value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }
}