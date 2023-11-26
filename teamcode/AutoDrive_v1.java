package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "A:AutoDrive_v1: Testing wheels")
public class AutoDrive_v1 extends LinearOpMode {

    // Control Hub - motors : GoBILDA 5203
    // A - BR - port 1
    // B - BL - port 3
    // Y - FR - port 0
    // X - FL - port 2

    // Extension Hub
    // port 0 - TR
    // port 1 - TL
    // port 2 - open
    // port 3 - TC
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor TL;
    private DcMotor TR;
    private DcMotor TC;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TC = hardwareMap.get(DcMotor.class, "TC");


        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

/*
            BL.setPower(1);
            BR.setPower(1);
            FL.setPower(1);
            FR.setPower(1);
*/

            //TL.setPower(-.5);
            //TR.setPower(-.5);
            //TC.setPower(-0.3);

            //TC.setPower(-0.2);

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("TL current position: " , TL.getCurrentPosition());
                telemetry.addData("TR current position: " , TR.getCurrentPosition());
                Utilities.Sleep(500);
                telemetry.update();
            }
        }
    }
}

