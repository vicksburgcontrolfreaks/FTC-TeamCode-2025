package org.firstinspires.ftc.teamcode.TestOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LEDManager;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="LED TEST", group="Tests")
public class LEDTest extends OpMode {
    RobotHardware hardware;
    LEDManager led;
    boolean lastB = false;
    int state = 0;
    boolean isRed = false;
    boolean tagFound = false;

    @Override public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        led = new LEDManager(hardware.leds, isRed);
        telemetry.addData("LED TEST", "DPAD UP = BLUE, DOWN = RED");
        telemetry.addData("LED TEST", "B = next state");
        telemetry.addData("LED TEST", "Looking for tag...");
    }

    @Override public void init_loop() {
        // Manual override
        if (gamepad2.dpad_up) isRed = false;
        if (gamepad2.dpad_down) isRed = true;

        // Auto-detect tag
        for (AprilTagDetection d : hardware.aprilTagProcessor.getDetections()) {
            if (d.id == 20) { isRed = false; tagFound = true; }
            if (d.id == 24) { isRed = true; tagFound = true; }
        }
        led.setIdle();
        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("Tag", tagFound ? "FOUND" : "SEARCHING");
        telemetry.update();
    }

    @Override public void start() {
        led = new LEDManager(hardware.leds, isRed);
    }

    @Override public void loop() {
        if (gamepad2.b && !lastB) {
            state = (state + 1) % 6;
            switch (state) {
                case 0: led = new LEDManager(hardware.leds, isRed); telemetry.addData("STATE", "IDLE"); break;
                case 1: led.startMatch(); telemetry.addData("STATE", "MATCH STARTED"); break;
                case 2: led.startSequence(); telemetry.addData("STATE", "SHOOTING"); break;
                case 3: led.endSequence(); telemetry.addData("STATE", "SHOT DONE"); break;
                case 4: led.setMagazineFull(true); telemetry.addData("STATE", "MAG FULL"); break;
                case 5: led = new LEDManager(hardware.leds, !isRed); telemetry.addData("STATE", "SWAP ALLIANCE"); break;
            }
        }
        lastB = gamepad2.b;

        led.update();
        telemetry.addData("State", state);
    }
}