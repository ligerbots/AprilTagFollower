// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tagfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TriggerShot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindAprilTagAndShoot extends SequentialCommandGroup {
  /** Creates a new FindAprilTagAndShoot. */
  public FindAprilTagAndShoot(DriveTrain driveTrain, ShooterPivot shooterPivot, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(driveTrain, shooterPivot, shooter);
    addCommands(
      new AprilTagFinder(driveTrain, shooterPivot),
      new TriggerShot(shooter)
    );
  }
}
