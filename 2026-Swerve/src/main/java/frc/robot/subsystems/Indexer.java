package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final SparkMax indexerMotor =
        new SparkMax(Constants.IDConstants.IndexerMotor_ID, MotorType.kBrushless);

    // Set motor speed
    public void runIndexer(double speed) {
        indexerMotor.set(speed);
    }

    // Stop motor
    public void stopIndexer() {
        indexerMotor.set(0);
    }

    // Command to run indexer at a fixed speed
    public Command runIndexerCommand(double speed) {
        return run(() -> runIndexer(speed));
    }

    // Command to stop indexer
    public Command stopIndexerCommand() {
        return run(this::stopIndexer);
    }
}