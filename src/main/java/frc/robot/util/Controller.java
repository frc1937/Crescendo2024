package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private final XboxController controller;
    private final int port;
    public enum Inputs {
        A(1), B(1), X(1), Y(1),
        LEFT_BUMPER(1), LEFT_STICK(1), RIGHT_BUMPER(1), RIGHT_STICK(1),
        BACK(1), START(1), DPAD_UP(1), DPAD_DOWN(1), DPAD_LEFT(1), DPAD_RIGHT(1),
        LEFT_X(6), LEFT_Y(7), RIGHT_X(1), RIGHT_Y(1);
        public final int id;

        Inputs(int id) {
            this.id = id;
        }
    }

    public Controller(int port) {
        this.port = port;
        this.controller = new XboxController(port);
    }

    /**
     * Returns a trigger that is active when the stick is pushed past 50%
     * @param button - the button to create a trigger for, must be a stick
     * @return a trigger that is active when the stick is pushed past 50%
     */
    public Trigger getStick(Inputs button) {
        return new Trigger(() -> controller.getRawAxis(button.id) > 0.5);
    }

    /**
     * Returns a trigger that is active when the button is pressed
     * @param button - the button to create a trigger for, must not be a stick
     * @return a trigger that is active when the button is pressed
     */
    public JoystickButton getButton(Inputs button) {
        return new JoystickButton(controller, button.id);
    }

    public double getRawAxis(Inputs input) {
        return DriverStation.getStickAxis(port, input.id);
    } //todo: LOL WTF test this PLEASE
}
