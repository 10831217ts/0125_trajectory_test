// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Subsystems.Drivetrain;

/** Add your docs here. */
public class RobotContainer {
    private Drivetrain drive = new Drivetrain();

    public Command getAutonomousCommand(){
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        //MaxVelocity & MaxAcceleration
        config.setKinematics(drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(new Pose2d(),new Pose2d(1.0,0, new Rotation2d())),
            //()沒東西等於零，上面means 從(0,0)走到(1.0,0) 角度0
            config   
        );


        RamseteCommand command = new RamseteCommand(
            trajectory, 
            drive.getPose(), 
            new RamseteController(2.0,0.7), 
            drive.getFeedforward(), 
            drive.getKinematics(), 
            drive.getspeeds(), 
            drive.getLeftPIDController(), 
            drive.getrightPIDController(), 
            drive::setOutput, 
            //why不用輸leftVolt rightVolt
            driv
            );


        return command;
    }
}
