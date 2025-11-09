package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.AlignAprilTag;

@Autonomous(name = "AUTO: 3-Shot Test", group = "Tests")
public class ThreeShotAuto extends LinearOpMode {

    private RobotHardware hardware;
    private AlignAprilTag shooter;
    private final ElapsedTime runtime = new ElapsedTime();

    private int shot = 0;                    // 0,1,2
    private double preVelocity = 0;
    private double postVelocity = 0;
    private int currentShot = 0;
    private final int SET_VELOCITY = 1600;   // Match your TeleOp

    private static final int INDEX_TICKS = 800;
    private static final double INDEX_POWER = 0.3;
    private static final int FLIP_TIME_MS = 250;

    private final double[] preVelocities = new double[3];
    private final double[] postVelocities = new double[3];

    @Override
    public void runOpMode() {
        // === INIT ===
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        shooter = new AlignAprilTag(hardware, null, telemetry);  // follower not needed

        hardware.flipper.setPosition(0.0);
        telemetry.addData("STATUS", "Initialized — Waiting for Start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // === START 3-SHOT SEQUENCE ===
        spinUpShooter();
        while (opModeIsActive() && shot < 3) {
            double vel = hardware.shooter.getVelocity();

            // Wait for shooter to spin up
            if (shot == 0 && vel > SET_VELOCITY - 25) {
                flipAndNext();  // Shot 1
            }
            // After shot 1 & 2: index then flip
            else if (shot > 0 && !hardware.collector.isBusy()) {
                flipAndNext();
            }
            // Index for shot 2 & 3
            else if (shot > 0 && hardware.collector.isBusy()) {
                // Just wait
            }
            // Initial spin-up
            else if (shot == 0) {
                telemetry.addData("Shooter", "Spinning up: %.0f / %d", vel, SET_VELOCITY);
            }

            telemetry.addData("Shot", shot);
            telemetry.addData("Collector Pos", hardware.collector.getCurrentPosition());
            telemetry.addData("Time", "%.1f sec", runtime.seconds());
            telemetry.update();
            sleep(20);  // Prevent overload
        }

        // === FINAL CLEANUP ===
        hardware.shooter.setPower(0);
        hardware.flipper.setPosition(0.0);
        telemetry.addData("AUTO", "3-Shot Sequence COMPLETE");
        telemetry.update();

        // === HOLD TELEMETRY UNTIL END OF AUTO TIME ===
        while (opModeIsActive()) {
            telemetry.addData("AUTO", "COMPLETE — Scroll up for shot data");
            telemetry.addData("Shot 1 Pre", "%.0f", getShotPre(1));
            telemetry.addData("Shot 1 Post", "%.0f", getShotPost(1));
            telemetry.addData("Shot 2 Pre", "%.0f", getShotPre(2));
            telemetry.addData("Shot 2 Post", "%.0f", getShotPost(2));
            telemetry.addData("Shot 3 Pre", "%.0f", getShotPre(3));
            telemetry.addData("Shot 3 Post", "%.0f", getShotPost(3));
            telemetry.update();
            sleep(50);
        }
    }

    private void spinUpShooter() {
        hardware.shooter.setVelocity(SET_VELOCITY);
        telemetry.addData("SPIN", "%d ticks/sec", SET_VELOCITY);
    }

    private void flipAndNext() {
        preVelocity = hardware.shooter.getVelocity();
        telemetry.addData("SHOT %d", shot + 1);
        telemetry.addData("  Pre", "%.0f", preVelocity);
        preVelocities[shot] = preVelocity;


        // Flip
        hardware.flipper.setPosition(0.5);
        sleep(FLIP_TIME_MS);
        hardware.flipper.setPosition(0.0);

        postVelocity = hardware.shooter.getVelocity();
        telemetry.addData("  Post", "%.0f", postVelocity);
        postVelocities[shot] = postVelocity;

        shot++;
        currentShot = shot;

        if (shot < 3) {
            indexAndWait();
        } else {
            hardware.shooter.setPower(0);
            hardware.flipper.setPosition(0);
        }
    }

    // ADD THESE TWO METHODS HERE
    private double getShotPre(int shotNum) {
        return (shotNum > 0 && shotNum <= 3) ? preVelocities[shotNum - 1] : 0;
    }

    private double getShotPost(int shotNum) {
        return (shotNum > 0 && shotNum <= 3) ? postVelocities[shotNum - 1] : 0;
    }

    private void indexAndWait() {
        int target = hardware.collector.getCurrentPosition() + INDEX_TICKS;
        hardware.collector.setTargetPosition(target);
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.collector.setPower(INDEX_POWER);
        telemetry.addData("INDEX", "+%d ticks", INDEX_TICKS);
    }


    // Optional: Add alignment
    // if (shot > 0) shooter.alignRotationOnly(20);
}