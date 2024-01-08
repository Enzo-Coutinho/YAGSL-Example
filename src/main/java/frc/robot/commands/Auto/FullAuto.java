// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import swervelib.parser.PIDFConfig;

/** Add your docs here. */
public class FullAuto extends BaseAutoBuilder {

    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final Subsystem[] driveRequirements;
    private PIDFConfig anglePIDController;
    private PIDFConfig yPIDController;
    private PIDFConfig xPIDController;
    private PIDController anglePIDControllerWPI;

    public FullAuto(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        PIDFConfig xPIDController,
        PIDFConfig yPIDController,
        PIDFConfig anglePIDController,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        Map<String, Command> eventMap,
        boolean useAllianceColor,
        Subsystem... driveRequirements) {
        super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);
  
      this.xPIDController = xPIDController;
      this.yPIDController = yPIDController;
      this.anglePIDController = anglePIDController;
      this.outputChassisSpeeds = outputChassisSpeeds;
      this.driveRequirements = driveRequirements;

      this.anglePIDControllerWPI = this.anglePIDController.createPIDController();
      this.anglePIDControllerWPI.enableContinuousInput(-180, 180);
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        // Trajetórias criada dentro da pasta PathPlanning em deploy
        return new PPSwerveControllerCommand(
            trajectory,
            poseSupplier,
            xPIDController.createPIDController(),
            yPIDController.createPIDController(),
            anglePIDControllerWPI,
            outputChassisSpeeds,
            useAllianceColor,
            driveRequirements);
    }

}
