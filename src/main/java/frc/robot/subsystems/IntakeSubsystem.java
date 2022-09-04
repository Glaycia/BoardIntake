// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;

public class IntakeSubsystem extends SubsystemBase {
    
    public static enum IntakeState {
        UP,
        MID,
        DOWN,
        BELOW
    }

    public WPI_TalonFX intakeActuatorMotor;

    private final double m_gearRatio = 14.25;
    private final double m_momentInertia = 0.1607;
    private final DCMotor m_Falcon500 = new DCMotor(12, 4.69, 257, 1.5, 40086.72, 1);
    private final LinearSystem<N2, N1, N1> m_feederArmPlant = LinearSystemId.createSingleJointedArmSystem(m_Falcon500, m_momentInertia, m_gearRatio);

    private final KalmanFilter<N2, N1, N1> m_armObserver =
      new KalmanFilter<N2, N1, N1>(
          Nat.N2(),
          Nat.N1(),
          m_feederArmPlant,
          VecBuilder.fill(3.0, 3), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // timestep
          0.020);

    private final LinearQuadraticRegulator<N2, N1, N1> m_armController = 
        new LinearQuadraticRegulator<>(m_feederArmPlant, 
        VecBuilder.fill(8, 2), //State Penalty Cost
        VecBuilder.fill(12), // Control Cost
        0.020); // timestep


    private final LinearSystemLoop<N2, N1, N1> m_controlLoop =
        new LinearSystemLoop<>(m_feederArmPlant, m_armController, m_armObserver, 12.0, 0.020);

    public void set(double position){
        m_controlLoop.setNextR(position);

        //Correct Kalman estimate with encoder data
        //m_controlLoop.correct(VecBuilder.fill(m_encoder.getRate()));
        m_controlLoop.predict(0.020);

        double nextVoltage = m_controlLoop.getU(0);
        intakeActuatorMotor.setVoltage(nextVoltage);
    }

    public IntakeSubsystem() {
        intakeActuatorMotor = new WPI_TalonFX(Constants.IntakeConstants.INTAKE_MOTOR);
    }

    @Override
    public void periodic() {

    }

    public static final double readingUp = 0;
    public static final double readingDown = 90;
    private static final double readingTolerance = 5;

    public double readIntakePosition(){
        return intakeActuatorMotor.getSelectedSensorPosition();
    }

    public IntakeState getIntakeState() {
        if(Math.abs(readIntakePosition() - readingUp) < readingTolerance){
            //Intake is up
            return IntakeState.UP;
        }else if(Math.abs(readIntakePosition() - readingDown) < readingTolerance){
            //Intake is down
            return IntakeState.DOWN;
        }else if(readIntakePosition() > readingUp && readIntakePosition() < readingDown){
            //Intake is transitioning bt up and down
            return IntakeState.MID;
        }else if(readIntakePosition() > readingDown){
            //Intake is getting shoved down
            return IntakeState.BELOW;
        }

        return IntakeState.UP;
    }

}