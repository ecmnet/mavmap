package com.comino.mavmap.map.map3D.impl.octomap.tools;

import java.util.List;

import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;

import georegression.struct.GeoTuple3D_F64;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.key.KeyRayReadOnly;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.tools.OcTreeRayTools;

public class MAVOctoMapTools {
	
	
	public static void insertPointCloudRays(List<GeoTuple3D_F64<?>> scan, GeoTuple3D_F64<?> sensorOrigin, MAVOccupancyOcTree tree, 
                double min, double max, boolean discretize)
	   {
	      if (scan.size() < 1)
	         return;

	      Vector3D direction = new Vector3D();
	      Point3D  origin    = new Point3D(sensorOrigin.x,sensorOrigin.y,-sensorOrigin.z);

	      for (int i = 0; i < scan.size(); i++)
	      {
	    	 GeoTuple3D_F64<?> p = scan.get(i);
	         Point3D point = new Point3D(p.x,p.y,-p.z);
	         direction.sub(point, origin);
	         double length = direction.norm();
	         if (min > 0.0 && length < min)
	            continue;

	         if (max > 0.0 && length > max)
	         {
	            point.scaleAdd(max / length, direction, origin);
	            KeyRayReadOnly ray = OcTreeRayTools.computeRayKeys(origin, point, tree.getResolution(), 16);
	            if (ray != null)
	            {
	               for (int j = 0; j < ray.size(); j++)
	                  tree.updateNode(ray.get(j), false); // insert freespace measurement
	            }
	         }
	         else
	         {
	            KeyRayReadOnly ray = OcTreeRayTools.computeRayKeys(origin, point, tree.getResolution(),  16);
	            if (ray != null)
	            {
	               for (int j = 0; j < ray.size(); j++)
	                  tree.updateNode(ray.get(j), false); // insert freespace measurement
	               tree.updateNode(point, true); // update endpoint to be occupied
	            }
	         }
	      }
	   }
	
	public static void addToPointCloud(PointCloud scan, GeoTuple3D_F64<?> point) {
		scan.add(new Point3D(point.x, point.y, -point.z));
		
	}

}
