package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleDisplay {

  public static HashMap <String, ShuffleboardTab> tabs = new HashMap<String, ShuffleboardTab>();
  public ShuffleDisplay() {} //I'm stuck on whether to find some use of each subsystem/command in this class or input the methods indiviually 
  public static void addTab(SubsystemBase subBase, String tabName) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    tabs.put(subBase.getName(), tab);
  }

  public static void addItem(String tabName, String itemName, BuiltInWidgets widgetType) {
    Shuffleboard.getTab(tabName).add(itemName, 0).withWidget(widgetType).withProperties(Map.of()).getEntry();
    tabs.put(tabName, Shuffleboard.getTab(tabName));
  }
}
