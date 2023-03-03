package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChenryLib.PID;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class shootNode1 extends CommandBase{

    private final Intake m_Intake;
    private final Swerve m_Swerve;
    private final ApriltagSubsystem m_apriltag;

    public shootNode1(Intake intake, ApriltagSubsystem apriltag, Swerve swerve){
        m_Intake = intake;
        m_apriltag = apriltag;
        m_Swerve = swerve;

        addRequirements(intake, apriltag, swerve);
    }

    Timer m_clock = new Timer();


    Transform3d cameraToApriltag, intakeToNode;
    /**
     * 😱😱😱😱:D</p>
     * x->robot's front</p>
     * y->robot's left</p>
     * z->robot's up
     */
    Transform3d IntakeToCamera = new Transform3d();
      /**
     * 😱😱😱😱:D</p>
     * x->robot's front</p>
     * y->robot's left</p>
     * z->robot's up
     */
    Transform3d apriltagToNode1 = new Transform3d(new Translation3d(0.48265, 0, 0.70545), new Rotation3d());

    double intakeTheta, distanceToNode, shootHeight, vertical_v, horizontal_v, shootOutput, chasisTurn, intakeMove;
    double adjustConstant1 = 0.1;
    double adjustConstant2 = 0.01;
    double adjustConstant3 = 0.1;
    double adjustConstant4 = 0.1;


    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        cameraToApriltag = m_apriltag.getCameratoTarget();
        intakeToNode = IntakeToCamera.plus(cameraToApriltag).plus(apriltagToNode1);
        distanceToNode = Math.sqrt(Math.pow(intakeToNode.getX(), 2) + Math.pow(intakeToNode.getY(), 2));
        shootHeight = intakeToNode.getZ();
        vertical_v = Math.sqrt(9.8*distanceToNode*distanceToNode/(2*intakeToNode.getZ())) + m_Swerve.getForwardVelocity();
        horizontal_v = Math.sqrt(2*9.8*intakeToNode.getZ());
        intakeTheta = Units.radiansToDegrees(Math.atan(horizontal_v/vertical_v));//不確定這是不是反三角函數😥😥😥+不確定他是弧度還是角度XD
        shootOutput = Math.sqrt(vertical_v*vertical_v + horizontal_v*horizontal_v)*adjustConstant1;
        chasisTurn = intakeToNode.getY();
        intakeMove = Units.rotationsToDegrees(m_Intake.getIntakeEncoder());
        //PID intakeMotorPID = new PID(0.001, 0, 0, intakeTheta, 1);
        PID turnPID = new PID(0.001, 0, 0, 0, 1);

        m_Intake.grab();
        //m_Intake.setIntakeMotor(intakeMotorPID.calculate(intakeMove));
        m_Swerve.drive(0, 0, turnPID.calculate(chasisTurn*adjustConstant2), false);
    }

    @Override
    public boolean isFinished() {

        if(Math.abs(intakeMove) <= intakeTheta + adjustConstant3 && Math.abs(chasisTurn) <= adjustConstant4){
            if(m_clock.advanceIfElapsed(0.5)){
                m_Intake.setIntakeShooter(-0.2);
            }
            if(m_clock.advanceIfElapsed(1)){
                m_Intake.setIntakeShooter(shootOutput);
            }
            return true;
       }else{
        return false;
       }
    }

    @Override
    public void end(boolean interrupted) {
        m_Intake.setIntakeMotor(0);
        m_Intake.setIntakeShooter(0);
    }
    

}
