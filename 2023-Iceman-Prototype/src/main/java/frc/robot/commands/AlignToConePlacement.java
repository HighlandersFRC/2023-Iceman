// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToConePlacement extends SequentialCommandGroup {
  /** Creates a new AlignToConePlacement. */
  public AlignToConePlacement(Drive drive, Peripherals peripherals, Lights lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive);
    addCommands(new SetVisionAlignmentPipeline(peripherals),
                new WaitCommand(0.5),
                new VisionAlignment(drive, peripherals, lights)
    );
  }
}
