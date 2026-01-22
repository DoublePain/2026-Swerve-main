package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class ShooterSubsystem extends SubsystemBase {
    
   


    // Laser can for detecting coral and Outtake motors
    private SparkMax m_LeftShooter;
    private SparkMax m_RightShooter;
    

    public enum ShooterState {
        NONE,
        SCORE
    }

    private ShooterState mState = ShooterState.NONE;

    public ShooterSubsystem() {
        // Initialize hardware
        m_LeftShooter = new SparkMax(Constants.IDConstants.Shooter_Left_ID, MotorType.kBrushless);
        m_RightShooter = new SparkMax(Constants.IDConstants.Shooter_Right_ID, MotorType.kBrushless);
       

      
    }

  

    // Method to set the speed of both motors
    public void setSpeed(double speed) {
        m_LeftShooter.set(speed);
        m_RightShooter.set(speed);
    }


    public void StopShoot() {
        mState = ShooterState.NONE;
        m_LeftShooter.set(0);
        m_RightShooter.set(0);
    }
    public Command StopShootCommand(){
        return run(() -> StopShoot());
    }
    //index
    public void Shoot() {
        mState = ShooterState.SCORE;
        setSpeed(Constants.ShooterConstants.Shooter_Speed);
    }
    public Command ShootCommand(){
        return run(() -> Shoot());
    }



  

    // Getter for the current state
    public ShooterState getState() {
        return mState;
    }

    // Output telemetry to the dashboard
    public void outputTelemetry() {

        SmartDashboard.putNumber("LeftShooterRPM", m_LeftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("RightShooterRPM", m_RightShooter.getEncoder().getVelocity());
    }


}

