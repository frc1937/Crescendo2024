package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Controller;
import org.junit.jupiter.api.Test;

import java.util.function.DoubleSupplier;

import static org.junit.jupiter.api.Assertions.assertEquals;

class ButtonTest {

    @Test
    void testButton() {
        // Test all buttons
        Controller controller = new Controller(0);

        //For all buttons, check if output from Controller.java is the same as from JoystickButton with normal controller method.

        Trigger aButton = controller.getButton(Controller.Inputs.A);
        JoystickButton validAButton = new JoystickButton(controller.getXboxController(), XboxController.Button.kA.value);

        Trigger stick = controller.getStick(Controller.Stick.LEFT_STICK);
        JoystickButton validStick = new JoystickButton(controller.getXboxController(), XboxController.Axis.kLeftTrigger.value);

        DoubleSupplier leftXAxis = () -> controller.getRawAxis(Controller.Axis.LEFT_X);
        DoubleSupplier validLeftXAxis = () -> controller.getXboxController().getRawAxis(XboxController.Axis.kLeftX.value);

        assertEquals(aButton.getAsBoolean(), validAButton.getAsBoolean());
        assertEquals(stick.getAsBoolean(), validStick.getAsBoolean());
        assertEquals(leftXAxis.getAsDouble(), validLeftXAxis.getAsDouble());
    }
}
