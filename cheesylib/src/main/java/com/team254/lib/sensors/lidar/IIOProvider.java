/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2015. All Rights Reserved.                        */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Purple Wave!              */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. Any        */
/* modifications to this code must be accompanied by the \License.txt file    */ 
/* in the root directory of the project.                                      */
/*----------------------------------------------------------------------------*/
package com.team254.lib.sensors.lidar;

interface IIOProvider {
    public boolean  isConnected();
    public void     run();
    public void     stop();
    public double getUpdateCount();
    public void	    enableLogging(boolean enable);
    public boolean  isInitialized();
}
