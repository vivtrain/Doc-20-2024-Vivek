// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.io.Controllers;

import java.util.HashSet;

public class ControllerRegistrar {

    HashSet<Integer> m_portsUsed = new HashSet<Integer>();

    private static ControllerRegistrar m_instance;
    public static ControllerRegistrar getInstance() {
        if (m_instance == null) {
            m_instance = new ControllerRegistrar();
        }
        return m_instance; 
    }
    private ControllerRegistrar() {}

    public void registerController(int port) {
        assert !m_portsUsed.contains(port);
        m_portsUsed.add(port);
    }

}
