// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotionMagicPivot extends SubsystemBase {
  
  private double kGearing = 20 * 36d/16;

  private final TalonFX pivotMotor = new TalonFX(0);
  private final TalonFXSimState pivotMotorSim = pivotMotor.getSimState();
  private final SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      DCMotor.getFalcon500(1), 
      kGearing, 
      0.04,
      0.225583,
      Units.degreesToRadians(-45),
      Units.degreesToRadians(250),
      true,
      Units.degreesToRadians(-10),
      null
    );

  private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(-10));

  //Unit is degrees per second.
  public MotionMagicPivot() {

    var pivotMotorConfigurator = pivotMotor.getConfigurator();
    var pivotMotorConfigs = new TalonFXConfiguration();

    var slot0Configs = pivotMotorConfigs.Slot0;
    slot0Configs.kV = 6;
    slot0Configs.kP = 0; //Volts per radian of position error
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kG = 0.16;

    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    var feedBackConfigs = pivotMotorConfigs.Feedback;

    var motionMagicConfig = pivotMotorConfigs.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 2.35;
    motionMagicConfig.MotionMagicAcceleration = 5;
    pivotMotorConfigurator.apply(pivotMotorConfigs);

    pivotMotor.setPosition(Units.degreesToRotations(-10));

  }

  public void simulate(){
    pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    armSim.setInputVoltage(pivotMotorSim.getMotorVoltage());

    armSim.update(0.02);

    final double position_rot = Units.radiansToRotations(armSim.getAngleRads());
    final double velocity_rps = (armSim.getVelocityRadPerSec()); 

    pivotMotorSim.setRawRotorPosition(position_rot);
    pivotMotorSim.setRotorVelocity(velocity_rps);

  }

  public void overridePos(double newPos){
    armSim.setState(0, 0);
  }

  public void changeSetpoint(double setpoint){
    voltageRequest.Position = setpoint;
  }

  @Override
  public void periodic() {
    pivotMotor.setControl(voltageRequest);
    //pivotMotor.setVoltage(0.16);
    Logger.recordOutput("Position", pivotMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Position Degrees", Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput("Raw Setpoint", voltageRequest.Position);
    Logger.recordOutput("Error", pivotMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Profiled Setpoint", pivotMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput("Velocity", pivotMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Pose3D", get3DPose());

  }

  public Pose3d get3DPose() {
    return new Pose3d(
        -0.32385, 0, 0.6312130886, new Rotation3d(0, -Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble())-Units.degreesToRadians(10), 0));
  }

}
