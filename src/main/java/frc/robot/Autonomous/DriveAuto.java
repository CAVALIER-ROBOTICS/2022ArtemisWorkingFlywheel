// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystems;

/** Add your docs here. */
public class DriveAuto {

    DriveTrainSubsystems driveSub;
    PathPlannerTrajectory simplePath;
    PathPlannerTrajectory complexPath;
    ProfiledPIDController thetaController;

    public DriveAuto(DriveTrainSubsystems d) {
       
        driveSub = d;
        thetaController = new ProfiledPIDController(Constants.ThetaController, .6, .1, Constants.thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ArrayList<PathPlannerTrajectory> loadTrajectories() {
        //PathPlannerTrajectory[] trajectoryList = {};
        ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();
        for (int i = 0; i < Constants.autonFiles.length; i++) {
            System.out.println(i);

          trajectoryList.add(i, PathPlanner.loadPath(Constants.autonFiles[i], 2, 4));
        }
        return trajectoryList;
    }
    //lol12
    public Command getPathAuto(int desiredPath) {
        System.out.println("Hehehehe  :)");
        ArrayList<PathPlannerTrajectory> loadedTrajectories = loadTrajectories();
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
       loadedTrajectories.get(desiredPath),
        driveSub::getPose,
        Constants.m_kinematics,
        new PIDController(Constants.XController, 0, 0),
        new PIDController(Constants.YController, 0, 0),
        thetaController,
        driveSub::setModules,
        driveSub); 

        // Run path following command, then stop at the end.
        return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0))).beforeStarting(
            new InstantCommand(() -> driveSub.resetOdometry(loadedTrajectories.get(desiredPath).getInitialPose())));
    }

}