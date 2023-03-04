package org.firstinspires.ftc.teamcode.Commands.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

public class liftCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private String pole;
    private ElapsedTime timer;
    private double timeout;

    public liftCommand(LiftSubsystem subsystem, String pole, double timeout) {
        this.timeout = timeout;
        this.pole = pole;
        liftSubsystem = subsystem;
        addRequirements(liftSubsystem);
    }


    @Override
    public void initialize() {
        liftSubsystem.setTarget(pole);
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
    }

    @Override
    public boolean isFinished() {
        return liftSubsystem.isReached() || timer.milliseconds() > timeout;
    }

}
