package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Intake;

public class intakeGrab extends CommandBase{
    private final Intake m_intake;
    private final Joystick m_Joystick;

    public intakeGrab(Intake mIntake, Joystick mJoystick){
      m_intake = mIntake;
      m_Joystick = mJoystick;
      addRequirements(mIntake);  
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_intake.release();
    }

    @Override
    public boolean isFinished() {
        if(m_Joystick.getRawButtonPressed(JoystickConstants.btn_X)){
            return false;
        }else{
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.grab();
    }
}
