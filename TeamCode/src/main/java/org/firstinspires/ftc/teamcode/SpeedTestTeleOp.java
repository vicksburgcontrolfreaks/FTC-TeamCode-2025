package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="SPEED TEST", group="Tests")
public class SpeedTestTeleOp extends OpMode {
    RobotHardware hardware;
    ThreeShots shotSeq;
    private Follower follower;
    ElapsedTime debounce = new ElapsedTime();
    boolean lastUp = false, lastDown = false, lastB = false;
    private int tagId = 20;    // or 24 for red alliance
    int speed = 1500;                   // start speed
    int shot = 0;

    @Override public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        shotSeq = new ThreeShots(hardware);
        tagId = 20; // or 24 for red alliance
        telemetry.addData("SPEED TEST", "DPAD UP/DOWN = ±50 | B = fire");
        telemetry.addData("Speed", speed);
    }

    @Override public void loop() {
        // DPAD UP/DOWN — debounced
        if (gamepad2.dpad_up && !lastUp && debounce.milliseconds() > 200) {
            speed = Math.min(speed + 50, 4000);
            debounce.reset();
        }
        if (gamepad2.dpad_down && !lastDown && debounce.milliseconds() > 200) {
            speed = Math.max(speed - 50, 0);
            debounce.reset();
        }
        lastUp = gamepad2.dpad_up;
        lastDown = gamepad2.dpad_down;

        // B = fire 3 shots
        if (gamepad2.b && !lastB) {
            shotSeq.start(speed);
        }
        shotSeq.update(tagId);
        lastB = gamepad2.b;


        // TELEMETRY
        telemetry.addData("Set Speed", speed);
        telemetry.addData("Shot", shotSeq.getCurrentShot());
        telemetry.addData("Pre", "%.0f", shotSeq.getPreVelocity());
        telemetry.addData("Post", "%.0f", shotSeq.getPostVelocity());

        double reported = 0;
        for (AprilTagDetection d : hardware.aprilTagProcessor.getDetections()) {
            if (d.id == tagId) {
                reported = d.ftcPose.z;
                break;
            }
        }
        int autoSpeed = getSpeedForDistance(reported);
        telemetry.addData("Auto Speed", autoSpeed);
    }

    private int getSpeedForDistance(double reported) {
        double real = reported * 1.47;
        if (real < 18) return 1800;
        if (real < 30) return 1400;
        return 1100;
    }
}