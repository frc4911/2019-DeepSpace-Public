package com.team4911.frc2019.utils;

import java.lang.reflect.AnnotatedType;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.controlboard.LeftDriveController;
import com.team4911.frc2019.controlboard.OperatorController;
import com.team4911.frc2019.controlboard.RightDriveController;

public class InfoDumper {

    private static String info = null;

    public void dumpInfo(){
        if (info == null) {
            info = "";
            info += getManifest();
            info += portMapping();
            info += buttonMappings();
            info += "Robot name is "+RobotName.name+"\n";
        }
        System.out.println(info);
    }

    private String getManifest(){
        ManifestManager m = new ManifestManager();
        String info = "---------Manifest----------\n";
        info += m.getManifestString() + "\n";
        return info;
    }

    private String portMapping(){
        String info = "---------Port-Mappings---------\n";
        info += controllerMapper (Constants.class);
        return info;
    }

    private String buttonMappings() {
        String info = "----------Button-Mappings---------\n";

        info += controllerMapper (LeftDriveController.class);
        info += controllerMapper (RightDriveController.class);
        info += controllerMapper (OperatorController.class);
        
        return info;
    }

    private String controllerMapper (Class theClass){
        String info = "";

		for(Constructor construct : theClass.getConstructors()) {
			Annot4911 annot = (Annot4911)construct.getAnnotation(Annot4911.class);
			if(annot != null) {
				info += "------"+annot.description()+"-----\n";
			}
		}

		for(Method method : theClass.getMethods()) {
			Annot4911 annot = (Annot4911)method.getAnnotation(Annot4911.class);
			if(annot != null) {
				info += annot.description() +"\n";
			}
		}

		for(Field field : theClass.getFields()) {
			Annot4911 annot = (Annot4911)field.getAnnotation(Annot4911.class);
			if(annot != null) {
				info += annot.description() +"\n";
			}
		}

        return info;
    }
}