package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Test", group = "Test")
public class LiftTest extends OpMode {

    private RobotHardware hardware;

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
        // Manual lift control
        if (gamepad2.a) {
            // A = Up
            hardware.lWinch.setPower(-1.0);
            hardware.rWinch.setPower(-1.0);
        } else if (gamepad2.y) {
            // Y = Down
            hardware.lWinch.setPower(1.0);
            hardware.rWinch.setPower(1.0);
        } else {
            // No button = Stop
            hardware.lWinch.setPower(0.0);
            hardware.rWinch.setPower(0.0);
        }

        // Telemetry
        telemetry.addData("Left Winch Encoder", hardware.lWinch.getCurrentPosition());
        telemetry.addData("Right Winch Encoder", hardware.rWinch.getCurrentPosition());
        telemetry.addData("Left Winch Power", hardware.lWinch.getPower());
        telemetry.addData("Right Winch Power", hardware.rWinch.getPower());
        telemetry.addData("", "");
        telemetry.addData("Controls", "A = Up (0.1), Y = Down (-0.1)");
        telemetry.update();
    }
}
