// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto01 extends SequentialCommandGroup {
  /** Creates a new auto. */
  public Auto01(Swerve s_sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    System.out.println("hats");
    addCommands(
      new TeleopSwerve(
                s_sSwerve, 
                () -> (0.4),//-driver.getRawAxis(translationAxis), 
                () -> (0.0),//-driver.getRawAxis(strafeAxis), 
                () -> (0.0),//-driver.getRawAxis(rotationAxis), 
                () -> (false)
            ).withTimeout(3)
    );
  }
}
