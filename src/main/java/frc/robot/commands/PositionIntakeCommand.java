package frc.robot.commands;


import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

public class PositionIntakeCommand extends CommandBase{
    private IntakeSubsystem m_intake;

    //private BooleanSupplier m_triggerDown;
    //States Inputs Outputs
    
    
    public void execute(){
        //Condition BELOW/protective ovverride TBA

        SmartDashboard.putNumber("Encoder Reading Value", m_intake.readIntakePosition());
        // if(m_triggerDown.getAsBoolean()){
        //     m_intake.set(IntakeSubsystem.readingDown);
        // }else{
        //     m_intake.set(IntakeSubsystem.readingUp);
        // }

    }
    //, BooleanSupplier triggerSupplier
    public PositionIntakeCommand(IntakeSubsystem intakeSub) {
        m_intake = intakeSub;
        //m_triggerDown = triggerSupplier;
        addRequirements(intakeSub);

    }

}
