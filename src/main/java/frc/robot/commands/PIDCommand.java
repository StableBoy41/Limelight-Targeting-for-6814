package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

import java.util.concurrent.ExecutorService;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class PIDCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final DriveTrainSubsystem drivetrainSubsystem;
    private final double setpoint;
    private final PIDController myPID = new PIDController(Constants.kp, Constants.ki, Constants.kd); 

    public PIDCommand(DriveTrainSubsystem subsystem, double setpoint) {
      this.drivetrainSubsystem = subsystem;
      this.setpoint = setpoint;
      addRequirements(subsystem);
    }
  
    @Override
    public void initialize() {
      drivetrainSubsystem.noSafe();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.SetMotors(myPID.calculate(drivetrainSubsystem.getLeftEncoder().getDistance() * Constants.kencoderTickToFeet, setpoint), myPID.calculate(drivetrainSubsystem.getLeftEncoder().getDistance() * Constants.kencoderTickToFeet, setpoint));
    }
  }
