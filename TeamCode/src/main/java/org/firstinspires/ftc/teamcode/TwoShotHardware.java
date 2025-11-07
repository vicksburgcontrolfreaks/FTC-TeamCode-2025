package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="2-SHOT HARDWARE", group="Tests")
public class TwoShotHardware extends OpMode {
    RobotHardware hardware;
    boolean bPressed = false;
    int shot = 0;               // 0 = first, 1 = second

    @Override public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        telemetry.addData("2-SHOT", "Press Play → B");
    }

    @Override public void loop() {
        // B = start burst
        if (gamepad2.b && !bPressed) {
            bPressed = true;
            shot = 0;
            hardware.shooter.setVelocity(1600);
            telemetry.addData("SPIN", "1600 ticks/sec");
        }

        double vel = hardware.shooter.getVelocity();
        telemetry.addData("Shooter", "%.0f / 1600", vel);

        // WAIT FOR SPEED
        if (bPressed && vel > 1500) {
            if (shot == 0) {
                // SHOT 1 – flip only
                flipAndNext();
            } else {
                // SHOT 2 – index +500 then flip
                int target = hardware.collector.getCurrentPosition() + 500;
                hardware.collector.setTargetPosition(target);
                hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                hardware.collector.setPower(0.9);
                telemetry.addData("INDEX", "+500 to " + target);
                bPressed = false;          // wait for index to finish
            }
        }

        // WHEN INDEX DONE → flip
        if (!bPressed && shot > 0 && shot < 3 && !hardware.collector.isBusy()) {
            flipAndNext();
        }

        telemetry.addData("Shot", shot + 1);
        telemetry.addData("Collector Pos", hardware.collector.getCurrentPosition());
    }

    private void flipAndNext() {
        hardware.flipper.setPosition(0.5);
        sleep(250);
        hardware.flipper.setPosition(0.0);
        telemetry.addData("FLIP", "Shot " + (shot + 1));
        shot++;
        if (shot < 3) {
            bPressed = true;           // trigger next shot
        } else {
            hardware.shooter.setPower(0);
            telemetry.addData("DONE", "2 shots fired!");
        }
    }

    private void sleep(long ms) { try { Thread.sleep(ms); } catch(Exception e) {} }
}