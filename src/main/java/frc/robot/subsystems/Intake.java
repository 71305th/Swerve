package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private final CANSparkMax intakeMotor = new CANSparkMax(61, MotorType.kBrushless);

    private final CANSparkMax intakeLeftShooter = new CANSparkMax(39, MotorType.kBrushless);
    private final CANSparkMax intakeRightShooter = new CANSparkMax(55, MotorType.kBrushless);

    private final MotorControllerGroup intakeShooter = new MotorControllerGroup(intakeLeftShooter, intakeRightShooter);

    
    

    public Intake() {
        intakeMotor.setInverted(false);
        intakeLeftShooter.setInverted(false);
        intakeRightShooter.setInverted(true);

        resetEncoders();
    }

    public void grab() {
        solenoid.set(true);
    }

    public void release() {
        solenoid.set(false);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public void resetEncoders() {
        intakeMotor.getEncoder().setPosition(0);
    }

    public void setIntakeShooter(double speed){
        intakeShooter.set(speed);
    }

    public double getIntakeEncoder(){
        return intakeMotor.getEncoder().getPosition();
    }

    public double getIntakeShooterVelocity(){
        return intakeShooter.get();
    }
}

