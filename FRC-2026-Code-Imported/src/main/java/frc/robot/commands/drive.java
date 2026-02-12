package frc.robot.commands;
import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class drive extends Command {
        private DriveTrain intakesub;
        private XboxController driver;
        public drive(DriveTrain intakesub,XboxController controller){
            this.intakesub = intakesub;
            addRequirements(intakesub);
            driver = controller;
        }
        @Override
        public void execute(){
            
            intakesub.Drive(driver.getLeftY(), driver.getRightY());
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Drive(0, 0);
        }
}
