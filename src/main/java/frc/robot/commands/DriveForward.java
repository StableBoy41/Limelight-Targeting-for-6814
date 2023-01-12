package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;

public class DriveForward extends CommandBase {
    private DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
    private double startTime;
    private double currentTime;
    private double setTime;

    public DriveForward(DriveTrainSubsystem driveTrainSubsystem, double setTime) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.setTime = setTime;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        driveTrainSubsystem.SetMotors(Constants.kForwardPower, Constants.kForwardPower);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.SetMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        if(currentTime - startTime > setTime) {
            return true;
        } else {
            return false;
        }
    }

}
