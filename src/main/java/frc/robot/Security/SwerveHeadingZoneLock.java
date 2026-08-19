package frc.robot.Security;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class SwerveHeadingZoneLock {
    private final CommandSwerveDrivetrain m_drivetrain;


    public double gyro = 0;

    private final PIDController m_headingPID;
    private boolean m_isOverrideActive = false;

    public SwerveHeadingZoneLock(CommandSwerveDrivetrain drivetrain  ) {
        this.m_drivetrain = drivetrain;


        
        // Aumentei um pouco o P (de 1.0 para 5.0) para que o giro seja mais agressivo e preciso.
        // Se o robô oscilar muito, diminua esse valor.
        this.m_headingPID = new PIDController(5.0, 0, 0.1); 
        this.m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Define se o botão de trava está pressionado.
     */
    public void setOverride(boolean active) {
        this.m_isOverrideActive = active;
    }


    /**
     * Agora o Lock está ativo sempre que o botão for pressionado, 
     * independente de onde o robô esteja no campo.
     */
    public boolean isLockActive() {
        return m_isOverrideActive;
    }

    public double calculateLockOmega(double maxRotation) {
        if (!isLockActive()) return 0.0;

        // Ângulo atual do robô no campo
        double currentRot = m_drivetrain.getState().Pose.getRotation().getRadians();
        
        // Pega o ângulo de mira que o shooter calculou (já com compensação de movimento)
        Rotation2d targetRotation = new Rotation2d(gyro);
        
        if (targetRotation == null) return 0.0;

        double targetRot = targetRotation.getRadians() ;
        


        // Calcula a velocidade de rotação necessária
        double output = m_headingPID.calculate(currentRot, targetRot );
        
        // Limita a velocidade para não ultrapassar o máximo permitido pelo chassi
        return edu.wpi.first.math.MathUtil.clamp(output, -maxRotation, maxRotation);
    }
}