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

public class DriveTrain  extends SubsystemBase {

    private SparkMax motor1;
    private SparkMax motor2;
    private SparkMax motor3;
    private SparkMax motor4;





    public DriveTrain() {
        motor1 = new SparkMax(1, MotorType.kBrushed);
        motor2 = new SparkMax(2, MotorType.kBrushed);
        motor3 = new SparkMax(3, MotorType.kBrushed);
        motor4 = new SparkMax(4, MotorType.kBrushed);
        
      
       
    }

public void Drive(double left, double right)
 {
    motor2.set(-right);
    //motor3.set(-right);

    motor1.set(-left);
    //motor4.set(-left);
}

}
