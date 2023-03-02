package org.firstinspires.ftc.teamcode.draco.commandBase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.draco.commandBase.subsystem.clawSubsystem;

public class clawPosCommand extends CommandBase {

    public clawPosCommand(clawSubsystem intake, double timeout) {
        this.position = position;
        this.timeout = timeout;
        this.intake = intake;
        this.max_v = v;
        this.max_a = a;
        this.allowed_error = allowed_error;
        this.errorState = error;
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
            intake.newProfile(position, max_v, max_a);
        }

        // didnt reach extension in the allotted time, so set the subsystem to the error state specified
        if (timer.milliseconds() > timeout) {
            intake.state = errorState;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getPos() - position) < allowed_error || timer.milliseconds() > timeout;
    }

}
