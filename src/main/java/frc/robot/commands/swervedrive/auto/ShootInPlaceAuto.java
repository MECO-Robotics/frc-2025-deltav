package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.indexer.IndexingCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class ShootInPlaceAuto extends SequentialCommandGroup {

  public ShootInPlaceAuto(ShooterSubsystem shooter, IndexingSubsystem indexer) {
    addRequirements(shooter);

    addCommands(
    new ShooterCommand(shooter, Constants.Shooter.Presets.kLeftSpeaker, Constants.Shooter.Presets.kRightSpeaker).withTimeout(3),
    new IndexingCommand(indexer, Constants.Indexing.indexingSpeed),
    new WaitCommand(.5),
    new IndexingCommand(indexer, 0),
    new InstantCommand(shooter::disable)
    );
    
  }                       
    

}
