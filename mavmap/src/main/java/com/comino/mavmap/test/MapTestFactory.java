package com.comino.mavmap.test;

import com.comino.mavcom.model.DataModel;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavmap.map.map3D.impl.octomap.tools.MAVOctoMapTools;

import georegression.struct.GeoTuple3D_F64;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.jOctoMap.tools.OcTreeRayTools;
import us.ihmc.jOctoMap.tools.OcTreeSearchTools;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class MapTestFactory {

	/**
	 * Builds a virtual wall in front of the vehicle's current orientation
	 * 
	 * @param map
	 * @param model
	 * @param distance_m
	 */

	public static void buildWalls(MAVOctoMap3D map, DataModel model, float distance_m, float rel_altitude) {


		PointCloud scan = new PointCloud();
		GeoTuple3D_F64<?>   wall  = new Point3D_F64();

		int height = (int)((Math.random()*5+0.3f)/map.getResolution());


		Point3D_F32         pos   = new Point3D_F32();

		pos.x = model.state.l_x + (float)Math.cos(model.attitude.y) * distance_m;
		pos.y = model.state.l_y + (float)Math.sin(model.attitude.y) * distance_m;

		for(int k=-3; k<=3; k++) {
			for(int i=0;i<height;i++) {
				wall.x = pos.x + (float)Math.sin(-model.attitude.y) * map.getResolution()*k;
				wall.y = pos.y + (float)Math.cos(-model.attitude.y) *map.getResolution()*k;
				wall.z = -map.getResolution()*i-0.1f;
				MAVOctoMapTools.addToPointCloud(scan, wall);
			}
		}	

		map.getTree().insertPointCloud(scan, new Point3D( model.state.l_x, model.state.l_y, -model.state.l_z));

	}


	public static void buildWall(MAVOctoMap3D map, DataModel model, float distance_m, float rel_altitude) {

		final int total_height = 13;

		PointCloud scan = new PointCloud();
		
		GeoTuple3D_F64<?>   wall  = new Point3D_F64();
		Point3D_F32         pos   = new Point3D_F32();

		for(int j=total_height; j > 0; --j) {

			pos.x = model.state.l_x + (float)Math.cos(model.attitude.y) * distance_m * j/ total_height;
			pos.y = model.state.l_y + (float)Math.sin(model.attitude.y) * distance_m * j/ total_height;

			for(int k=-3; k<=3; k++) {
				for(int i=0;i<j;i++) {
					wall.x = pos.x + (float)Math.sin(-model.attitude.y) * map.getResolution()*k;
					wall.y = pos.y + (float)Math.cos(-model.attitude.y) * map.getResolution()*k;
					wall.z = -map.getResolution()*i-0.1f;
					MAVOctoMapTools.addToPointCloud(scan, wall);
				}
			}	

		}

		map.getTree().insertPointCloud(scan, new Point3D( model.state.l_x, model.state.l_y, -model.state.l_z));

	}


}
