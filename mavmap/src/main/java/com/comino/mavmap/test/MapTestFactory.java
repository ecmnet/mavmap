package com.comino.mavmap.test;

import com.comino.mavcom.model.DataModel;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.point.Point3D_F32;

public class MapTestFactory {

	/**
	 * Builds a virtual wall in front of the vehicle's current orientation
	 * 
	 * @param map
	 * @param model
	 * @param distance_m
	 */

	public static void buildWall(MAVOctoMap3D map, DataModel model, float distance_m, float rel_altitude) {

		if(map==null)
			return;
		
		map.disableRemoveOutdated();

		Point3D_F32   pos          = new Point3D_F32();
		Point3D_F32   wall         = new Point3D_F32();

		pos.x = model.state.l_x + (float)Math.cos(model.attitude.y) * distance_m;
		pos.y = model.state.l_y + (float)Math.sin(model.attitude.y) * distance_m;

		for(int k=-3; k<=3; k++) {
			for(int i=0;i<15;i++) {
				wall.x = pos.x + (float)Math.sin(-model.attitude.y) * map.getResolution()*k;
				wall.y = pos.y + (float)Math.cos(-model.attitude.y) * map.getResolution()*k;
				wall.z = -map.getResolution()*i-0.1f;
				map.insert(wall);
			}
		}	
	}

}
