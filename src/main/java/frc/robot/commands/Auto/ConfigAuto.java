// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants2.Dimensoes;
import frc.robot.Constants2.PID;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class ConfigAuto {
    List<PathPlannerTrajectory> pathGroup;
    FullAuto autonomo;

    public ConfigAuto(SwerveSubsystem swerve, boolean alianca, String nomeTrajetoria) {
        pathGroup = PathPlanner.loadPathGroup(nomeTrajetoria, new PathConstraints(Dimensoes.MAX_VEL_AUTO, Dimensoes.MAX_ACCE_AUTO));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("intake", new PrintCommand("Exemplo de Intake"));

        autonomo = new FullAuto(swerve::getPose, 
                            swerve::resetOdometry, 
                            PID.xAutoPID, 
                            PID.yAutoPID, 
                            PID.angleAutoPID, 
                            swerve::setChassisSpeeds, 
                            eventMap, 
                            alianca, 
                            swerve);
    }

    public Command returnCommand() {
        return autonomo.fullAuto(pathGroup);
    }
}
