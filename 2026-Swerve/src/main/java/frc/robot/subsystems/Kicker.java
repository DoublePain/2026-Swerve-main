package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class Kicker extends SubsystemBase{
    

     private final SparkMax Kicker = new SparkMax(15, MotorType.kBrushless);

// Method to set the speed of both motors
    public void setSpeed(double speed) {
        Kicker.set(speed);
 
    }

    public void Kick(double speed){
        Kicker.set(speed);
    }

public Command Kick () {
    return run(() -> Kick());
}
public void KickerStop() {
    
        Kicker.set(0);
    }
    public Command KickerStopCommand(){
        return run(() -> KickerStop());
    }
}
