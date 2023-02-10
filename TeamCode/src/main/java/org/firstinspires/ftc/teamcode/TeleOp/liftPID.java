package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@Config
//@Disabled
@TeleOp(name = "liftPID", group = "Iterative Opmode")
public class liftPID extends OpMode {

    HWMap drive;

    //PID
    PIDController liftController;
    public static int liftTarget = 550;
    public static double pL = 0.01, iL = 0, dL = 0;

    PIDController extendController;
    public static int extendTarget = 1000;
    public static double pE = 0.01, iE = 0, dE = 0;

    @Override
    public void init(){
        drive = new HWMap(hardwareMap);
        liftController = new PIDController(pL, iL, dL);
        extendController = new PIDController(pE, iE, dE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        //LIFT
        liftController.setPID(pL, iL, dL);
        int liftPos = drive.leftVerticalSlide.getCurrentPosition();
        double pid = liftController.calculate(liftPos, liftTarget);

        drive.leftVerticalSlide.setPower(pid);
        drive.rightVerticalSlide.setPower(pid);

        telemetry.addData("Lift Pos", liftPos);
        telemetry.addData("Lift Target", liftTarget);
        telemetry.update();

        //EXTEND
        extendController.setPID(pE, iE, dE);
        int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
        double pidExtend = extendController.calculate(extendPos, extendTarget);

        drive.leftHorizontalSlide.setPower(pidExtend);
        drive.rightHorizontalSlide.setPower(pidExtend);

        telemetry.addData("Extend Pos", extendPos);
        telemetry.addData("Extend Target", extendTarget);
        telemetry.update();
    }
}
