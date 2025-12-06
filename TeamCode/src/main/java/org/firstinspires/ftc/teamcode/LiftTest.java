package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Lift Test", group = "Test")
public class LiftTest extends OpMode {

    private RobotHardware hardware;
    private double power = 0.75;

    // Current spike detection parameters
    private static final double CURRENT_THRESHOLD = 5800.0; // milliamps (7A)
    private boolean currentSpikeDetected = false;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // Set lift motors to run without encoder for manual control
        hardware.lWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A = Up, Y = Down");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Check for current spike on both motors
        double leftCurrent = ((DcMotorEx) hardware.lWinch).getCurrent(CurrentUnit.AMPS);
        double rightCurrent = ((DcMotorEx) hardware.rWinch).getCurrent(CurrentUnit.AMPS);

        // Detect current spike
        if (leftCurrent > CURRENT_THRESHOLD || rightCurrent > CURRENT_THRESHOLD) {
            currentSpikeDetected = true;
            hardware.lWinch.setPower(0.0);
            hardware.rWinch.setPower(0.0);
        } else {
            currentSpikeDetected = false;
        }

        // Manual lift control (only if no current spike)
        if (!currentSpikeDetected) {
            if (gamepad2.a) {
                // A = Up
                hardware.lWinch.setPower(-power);
                hardware.rWinch.setPower(-power);
            } else if (gamepad2.y) {
                // Y = Down
                hardware.lWinch.setPower(power);
                hardware.rWinch.setPower(power);
            } else {
                // No button = Stop
                hardware.lWinch.setPower(0.0);
                hardware.rWinch.setPower(0.0);
            }
        }

        // Telemetry
        telemetry.addData("Left Winch Encoder", hardware.lWinch.getCurrentPosition());
        telemetry.addData("Right Winch Encoder", hardware.rWinch.getCurrentPosition());
        telemetry.addData("Left Winch Power", hardware.lWinch.getPower());
        telemetry.addData("Right Winch Power", hardware.rWinch.getPower());
        telemetry.addData("Left Current (mA)", "%.1f", leftCurrent);
        telemetry.addData("Right Current (mA)", "%.1f", rightCurrent);
        telemetry.addData("Current Spike", currentSpikeDetected ? "DETECTED - STOPPED!" : "Normal");
        telemetry.addData("Current Threshold (mA)", CURRENT_THRESHOLD);
        telemetry.addData("", "");
        telemetry.addData("Controls", "A = Up (0.1), Y = Down (-0.1)");
        telemetry.update();
    }
}
