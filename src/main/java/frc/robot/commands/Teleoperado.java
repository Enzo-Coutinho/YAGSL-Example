// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants2.Dimensoes;
import frc.robot.Constants2.Tracao;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** 
 * Classe que calcula a partir da entrada do gamepad a saída do swerve
 */
public class Teleoperado extends CommandBase {
    // Variáveis que guardam nossas funções do gamepad
    DoubleSupplier y;
    DoubleSupplier x;
    DoubleSupplier turn;

    // Objetos necessárias para acessar funções e variáveis
    SwerveSubsystem swerve;
    SwerveController controller;

    // Variáveis que guardam a translação e velocidade angular do swerve
    Translation2d translation;
    double angle;
    double omega;   

    public Teleoperado(SwerveSubsystem swerve, DoubleSupplier y, DoubleSupplier x, DoubleSupplier turn) {
      // Aqui atribuimos as funções e subsistema
      this.y = y;
      this.x = x;
      this.turn = turn;
      this.swerve = swerve;
      controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
      // Adiciona a tração como requerimento
      addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
   
    }

    // Abaixo calculamos os valores de saída a partir dos nossos inputs
    @Override
    public void execute() {
    double xVelocity   = y.getAsDouble() * Tracao.multiplicadorTranslacionalY;
    double yVelocity   = x.getAsDouble() * Tracao.multiplicadorTranslacionalX;
    double angVelocity = turn.getAsDouble() * Tracao.multiplicadorRotacional;
    
    ChassisSpeeds speeds = swerve.getTargetSpeeds(xVelocity, yVelocity);
    
    speeds = swerve.discretize(speeds);
    
    translation = SwerveController.getTranslation2d(speeds);

    omega = controller.config.maxAngularVelocity * angVelocity;
    
    // Caso essa função seja verdadeira a aceleração do robô será limitada
    if(Tracao.accelCorrection) {
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                              Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS, 
                                               List.of(Dimensoes.CHASSIS),
                                               swerve.getSwerveDriveConfiguration());
    }
    
    // Aqui temos nossa função definida dentro da classe de subsistema a qual comandara o swerve
    swerve.drive(translation, omega, Tracao.fieldRelative, Tracao.isOpenLoop, swerve.returnPIDCorrection());
    }

    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
