package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class BasicAutoCommand extends SequentialCommandGroup{
  public BasicAutoCommand(Drivetrain drivetrain){
    addRequirements(drivetrain);
    addCommands(


    );
  }
}