package com.comino.mavmap.map.map3D.impl.octomap.boundingbox;

import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;

public class MAVBoundingBox extends OcTreeBoundingBoxWithCenterAndYaw {
	
	private final OcTreeSimpleBoundingBox box = new OcTreeSimpleBoundingBox();
	
	public MAVBoundingBox() {
		super();
	}
	
	public MAVBoundingBox(GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		super();
		set(p,half_side_xy,half_side_z);
	}
	
	public MAVBoundingBox(GeoTuple4D_F32<?> p, float half_side_xy) {
		super();
		setTop(p,half_side_xy);
	}
	
	public void set(GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		box.setMinX( p.x-half_side_xy); box.setMaxX( p.x+half_side_xy);
		box.setMinY( p.y-half_side_xy); box.setMaxY( p.y+half_side_xy);
		box.setMinZ(-p.z-half_side_z);  box.setMaxZ(-p.z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(0.8f, 16);
	}
	
	public void setTop(GeoTuple4D_F32<?> p, float half_side_xyz) {
		box.setMinX( p.x-half_side_xyz); box.setMaxX( p.x+half_side_xyz);
		box.setMinY( p.y-half_side_xyz); box.setMaxY( p.y+half_side_xyz);
		box.setMinZ(-p.z-0.2f); box.setMaxZ(-p.z+half_side_xyz);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(0.2f, 16);
	}


}
