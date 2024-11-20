// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.io.Controllers;

import java.util.HashSet;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RegisteredController extends GenericHID {
    private ControllerRegistrar m_registrar = ControllerRegistrar.getInstance();

    private HashSet<Integer> m_buttonMappings = new HashSet<Integer>();
    private HashSet<Integer> m_axisMappings = new HashSet<Integer>();

    public RegisteredController(int port) {
        super(port);
        m_registrar.registerController(port);
    }

    public JoystickButton registerButtonMap(int buttonID) {
        if (m_buttonMappings.contains(buttonID))
            throw new RuntimeException("Multiple mappings on button " + Integer.toString(buttonID) + " of GenericHID " +  Integer.toString(super.getPort()));
        m_buttonMappings.add(buttonID);
        return new JoystickButton(this, buttonID);
    }

    public JoystickAxisTrigger registerTriggerMap(int axisID, double threshold) {
        if (m_axisMappings.contains(axisID))
            throw new RuntimeException("Multiple mappings on axis " + Integer.toString(axisID) + " of GenericHID " +  Integer.toString(super.getPort()));
        m_axisMappings.add(axisID);
        return new JoystickAxisTrigger(this, axisID, threshold);
    }

}
