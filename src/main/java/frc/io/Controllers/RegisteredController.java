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

  /** Registers the button as mapped and returns a Trigger to which a Command can be mapped
   * @param buttonID the button to be registered as used
   * @return JoystickButton corresponding to the mapping */
	public JoystickButton registerButtonMap(int buttonID) {
		if (m_buttonMappings.contains(buttonID))
			throw new RuntimeException("Multiple mappings on button " + Integer.toString(buttonID) + " of GenericHID " +  Integer.toString(super.getPort()));
		m_buttonMappings.add(buttonID);
		return new JoystickButton(this, buttonID);
	}

  /** Registers the axis as mapped and returns a Trigger to which a Command can be mapped
   * @param axisID the axis to be registered as used
   * @param threshold the threshold for triggering
   * @return JoystickAxisTrigger corresponding the mapping */
	public JoystickAxisTrigger registerAxisMap(int axisID, double threshold) {
		if (m_axisMappings.contains(axisID))
			throw new RuntimeException("Multiple mappings on axis " + Integer.toString(axisID) + " of GenericHID " +  Integer.toString(super.getPort()));
		m_axisMappings.add(axisID);
		return new JoystickAxisTrigger(this, axisID, threshold);
	}

  /** Registers a pair of axes as mapped and returns a Trigger to which a Command can be mapped
   * @param xAxisID the "horizontal" axis to be registered as used
   * @param yAxisID the "vertical" axis to be registerd as used
   * @param threshold the threshold for triggering (circular threshold i.e. norm is calculated)
   * @return Joystick2DAxisTrigger corresponding to the mapping */
	public Joystick2DAxisTrigger register2DAxisMap(int xAxisID, int yAxisID, double threshold) {
		if (m_axisMappings.contains(xAxisID))
			throw new RuntimeException("Multiple mappings on axis " + Integer.toString(xAxisID) + " of GenericHID " +  Integer.toString(super.getPort()));
		if (m_axisMappings.contains(yAxisID))
			throw new RuntimeException("Multiple mappings on axis " + Integer.toString(yAxisID) + " of GenericHID " +  Integer.toString(super.getPort()));
		m_axisMappings.add(xAxisID);
		m_axisMappings.add(yAxisID);
		return new Joystick2DAxisTrigger(this, xAxisID, yAxisID, threshold);
	}

}
