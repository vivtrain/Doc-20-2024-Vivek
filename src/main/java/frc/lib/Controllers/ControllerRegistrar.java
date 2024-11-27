// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Controllers;

import java.util.HashSet;

public class ControllerRegistrar {

  // Data structure to track which ports are used
  private HashSet<Integer> m_portsUsed = new HashSet<Integer>();

  // Singleton code pattern ensures only one registrar exists in program
  private static ControllerRegistrar m_instance;
  public static ControllerRegistrar getInstance() {
    if (m_instance == null)
      m_instance = new ControllerRegistrar();
    return m_instance;
  }

  // Do not instantiate directly
  private ControllerRegistrar() {}

  /** Register a port as used
   * @param port port to register
   * @throws RuntimeException when multiple controllers are mapped to the same port */
  public void registerController(int port) throws RuntimeException {
    if (m_portsUsed.contains(port))
      throw new RuntimeException("Multiple controllers mapped to port " + Integer.toString(port));
    m_portsUsed.add(port);
  }

}
