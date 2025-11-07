package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * EXACT COPY of ThreeShotSequence — but as a reusable class.
 * Use from any OpMode: new ThreeShots(hardware, follower).start(1600);
 */
public class ThreeShots {
    private final RobotHardware hardware;
    private final ShootAprilTag shooter;

    private boolean bPressed = false;
    private int shot = 0; // 0,1,2
    private double preVelocity = 0;
    private double postVelocity = 0;
    private int currentShot = 0;

    public ThreeShots(RobotHardware hardware) {
        this.hardware = hardware;
        this.shooter = new ShootAprilTag(hardware, null, hardware.telemetry);
    }

    public void start(int speed) {
        if (bPressed) return;
        bPressed = true;
        shot = 0;
        hardware.shooter.setVelocity(speed);
    }

    public void update(int tagId) {
        // AUTO-AIM ONLY WHILE SHOOTING
        if (bPressed || shot > 0) {
            shooter.alignRotationOnly(tagId);
        }

        double vel = hardware.shooter.getVelocity();

        // WAIT FOR SPEED
        if (bPressed && vel > 1500) {
            if (shot == 0) flipAndNext();
            else indexAndWait();
        }

        // WHEN INDEX DONE → flip
        if (!bPressed && shot > 0 && !hardware.collector.isBusy()) {
            flipAndNext();
        }
    }

    private void flipAndNext() {
        preVelocity = hardware.shooter.getVelocity(); // ← PRE-SHOT
        hardware.flipper.setPosition(0.5);
        sleep(250);
        hardware.flipper.setPosition(0.0);
        postVelocity = hardware.shooter.getVelocity(); // ← POST-SHOT
        currentShot = shot + 1;                        // ← SHOT NUMBER
        shot++;
        if (shot < 3) {
            bPressed = true;
            // COLLECTOR STAYS ON
        } else {
            // LAST SHOT — COLLECTOR KEEPS RUNNING
            hardware.shooter.setPower(0);
            // DO NOT STOP COLLECTOR HERE
        }
    }

    private void indexAndWait() {
        int target = hardware.collector.getCurrentPosition() + 500;
        hardware.collector.setTargetPosition(target);
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.collector.setPower(0.9);
        bPressed = false; // wait for isBusy()
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception e) {}
    }

    public double getPreVelocity() { return preVelocity; }
    public double getPostVelocity() { return postVelocity; }
    public int getCurrentShot() { return currentShot; }
    public boolean isBusy() { return bPressed || shot > 0; }
}