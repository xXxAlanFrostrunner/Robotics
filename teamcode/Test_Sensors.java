package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "TestSensors (Blocks to Java)")
public class Test_Sensors extends LinearOpMode {

    private DigitalChannel RightRearLimit;

    private DcMotor TL;
    private DcMotor TR;


    //True = Magnet Not On
    //Flase = Magnet On
    @Override
    public void runOpMode() {
        RightRearLimit = hardwareMap.get(DigitalChannel.class, "Right-Rear-Limit");

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                TL.setPower(gamepad2.right_stick_y);
                TR.setPower(gamepad2.right_stick_y);
                telemetry.addData("TL current position: " , TL.getCurrentPosition());
                telemetry.addData("TR current position: " , TR.getCurrentPosition());
                telemetry.update();
                if(!RightRearLimit.getState()){
                    TL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    TL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
                telemetry.addData("Limit", RightRearLimit.getState());
            }
        }
    }
}