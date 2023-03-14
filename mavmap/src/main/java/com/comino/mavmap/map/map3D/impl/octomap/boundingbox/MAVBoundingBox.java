package com.comino.mavmap.map.map3D.impl.octomap.boundingbox;

import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;

public class MAVBoundingBox extends OcTreeBoundingBoxWithCenterAndYaw {
	
	
	public MAVBoundingBox(GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		super();
		
		OcTreeSimpleBoundingBox box = new OcTreeSimpleBoundingBox();
		box.setMinX( p.x-half_side_xy); box.setMaxX( p.x+half_side_xy);
		box.setMinY( p.y-half_side_xy); box.setMaxY( p.y+half_side_xy);
		box.setMinZ(-p.z-half_side_z);  box.setMaxZ(-p.z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(0.2, 16);
	
	}


}
