package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="3-SHOT HARDWARE", group="Tests")
public class ThreeShotSequence extends OpMode {
    RobotHardware hardware;
    ShootAprilTag shooter;
    private Follower follower;
    boolean bPressed = false;
    int shot = 0;               // 0,1,2
    private double preVelocity = 0;
    private double postVelocity = 0;
    private int currentShot = 0;
    private int setVelocity = 1500;

    @Override public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        shooter = new ShootAprilTag(hardware, follower, telemetry);
        follower = Constants.createFollower(hardwareMap);
        telemetry.addData("3-SHOT", "Press Play → B");
    }

    @Override public void loop() {
        // AUTO-AIM ONLY WHILE SHOOTING
        if (bPressed || shot > 0) {            // ← THIS LINE
            shooter.alignRotationOnly(20);  // ← YOUR ALIGN METHOD
        }
        // ONE B = THREE SHOTS
        if (gamepad2.b && !bPressed) {
            bPressed = true;
            shot = 0;
            hardware.shooter.setVelocity(setVelocity);
            telemetry.addData("SPIN", "1600 ticks/sec");
        }

        double vel = hardware.shooter.getVelocity();
        telemetry.addData("Shooter", "%.0f / 1600", vel);

        // WAIT FOR SPEED
        if (bPressed && vel > setVelocity - 50) {
            if (shot == 0) flipAndNext();               // Shot 1
            else indexAndWait();                        // Shot 2 & 3
        }

        // WHEN INDEX DONE → flip
        if (!bPressed && shot > 0 && !hardware.collector.isBusy()) {
            flipAndNext();
        }

        telemetry.addData("Shot", shot + 1);
        telemetry.addData("Collector Pos", hardware.collector.getCurrentPosition());
    }

    private void flipAndNext() {
        preVelocity = hardware.shooter.getVelocity();   // ← PRE-SHOT
        hardware.flipper.setPosition(0.5);
        sleep(250);
        hardware.flipper.setPosition(0.0);
        postVelocity = hardware.shooter.getVelocity();  // ← POST-SHOT
        currentShot = shot + 1;                         // ← SHOT NUMBER
        telemetry.addData("SHOT %d", currentShot);
        telemetry.addData("  Pre", "%.0f", preVelocity);
        telemetry.addData("  Post", "%.0f", postVelocity);
        shot++;
        if (shot < 3) {
            bPressed = true;
            // COLLECTOR STAYS ON
        } else {
            hardware.shooter.setPower(0);
            hardware.flipper.setPosition(0.0);
        }
    }

    private void indexAndWait() {
        int target = hardware.collector.getCurrentPosition() + 500;
        hardware.collector.setTargetPosition(target);
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.collector.setPower(0.9);
        telemetry.addData("INDEX", "+500");
        bPressed = false;                 // wait for isBusy()
    }

    private void sleep(long ms) { try { Thread.sleep(ms); } catch(Exception e) {} }
    public double getPreVelocity() { return preVelocity; }
    public double getPostVelocity() { return postVelocity; }
    public int getCurrentShot() { return currentShot; }
}