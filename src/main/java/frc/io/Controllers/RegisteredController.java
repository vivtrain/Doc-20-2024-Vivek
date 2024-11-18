// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.io.Controllers;

import java.util.HashSet;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RegisteredController extends GenericHID {
    ControllerRegistrar m_controllerBase = ControllerRegistrar.getInstance();

    HashSet<Integer> m_buttonMappings = new HashSet<Integer>();
    HashSet<Integer> m_axisMappings = new HashSet<Integer>();

    public RegisteredController(int port) {
        super(port);
        m_controllerBase.registerController(port);
    }

    public JoystickButton registerButtonMap(int buttonID) {
        assert !m_buttonMappings.contains(buttonID);
        m_buttonMappings.add(buttonID);
        return new JoystickButton(this, buttonID);
    }

    public JoystickAxisTrigger registerTriggerMap(int axisID, double threshold) {
        assert !m_axisMappings.contains(axisID);
        m_axisMappings.add(axisID);
        return new JoystickAxisTrigger(this, axisID, threshold);
    }

}
