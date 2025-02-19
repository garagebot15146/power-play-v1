package org.firstinspires.ftc.teamcode.Commands.commands.Medium;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;

public class extendMediumCommand extends CommandBase {

    private final ExtendSubsystem extendSubsystem;
    private ElapsedTime timer;
    private double timeout;
    private int cones;

    public extendMediumCommand(ExtendSubsystem subsystem, int cones, double timeout) {
        this.cones = cones;
        this.timeout = timeout;
        extendSubsystem = subsystem;
        addRequirements(extendSubsystem);
    }


    @Override
    public void initialize() {
        switch (cones) {
            case 0:
                extendSubsystem.setTarget(5);
                break;
            case 1:
                extendSubsystem.setTarget(800);
                break;
            case 2:
                extendSubsystem.setTarget(800);
                break;
            case 3:
                extendSubsystem.setTarget(800);
                break;
            case 4:
                extendSubsystem.setTarget(800);
                break;
            case 5:
                extendSubsystem.setTarget(800);
                break;
            case 6:
                extendSubsystem.setTarget(extendSubsystem.position() - 25);
                break;
        }
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
    }


    @Override
    public boolean isFinished() {
        return extendSubsystem.isReached() || timer.milliseconds() > timeout;
    }

}
