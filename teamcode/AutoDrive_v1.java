package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "A:AutoDrive_v1: Testing wheels")
public class AutoDrive_v1 extends LinearOpMode {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FR;
    private DcMotor FL;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            BL.setPower(0.1);
            BR.setPower(0.1);
            FL.setPower(0.1);
            FR.setPower(0.1);

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
}

