package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@Config
@TeleOp(name = "extendPID", group = "Iterative Opmode")
//@Disabled
public class extendPID extends OpMode {

    static HWMap drive;

    double pidExtend = 0;

    //PID
    PIDController extendController;
    public int extendTarget;
    public static double pL = 0.04, iL = 0.001, dL = 0.0004;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        //COLOR SENSOR
//        drive.colorSensor.enableLed(true);

        //PID
        extendController = new PIDController(pL, iL, dL);
        extendController.setTolerance(5);

        //DASHBOARD
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //HORIZONTAL SLIDES
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.intakeAngle.setPosition(0.63);
        drive.clawAngle.setPosition(0.3);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // ENCODERS
        int extensionPos = drive.leftHorizontalSlide.getCurrentPosition();


        // GAMEPAD B
        extendController.setPID(pL, iL, dL);
        pidExtend = extendController.calculate(extensionPos, extendTarget);
        drive.leftHorizontalSlide.setPower(pidExtend);
        drive.rightHorizontalSlide.setPower(pidExtend);

        if(gamepad1.x){
            extendTarget = 990;
        }
        if(gamepad1.a){
            extendTarget = 5;
        }

        // Bottom Pole
        if (gamepad2.left_trigger > 0.05) {
            drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // TELEMETRY
        telemetry.addData("Extend Pos", extensionPos);

    }

    @Override
    public void stop() {
    }


}
