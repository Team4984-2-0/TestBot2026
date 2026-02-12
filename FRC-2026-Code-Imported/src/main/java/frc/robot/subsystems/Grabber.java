package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Grabber  extends SubsystemBase {

    private SparkMax motor5;
        private final RelativeEncoder motor2Encoder;




    public Grabber() {
        motor5 = new SparkMax(5, MotorType.kBrushed);

        motor2Encoder = motor5.getEncoder();
       
    }

public void Spin(double value) {
        

    motor5.set(value);
}

public void Spin() {
    motor5.set(0);
   
}

public double get_encoder(){
    return motor2Encoder.getPosition();
}

}
