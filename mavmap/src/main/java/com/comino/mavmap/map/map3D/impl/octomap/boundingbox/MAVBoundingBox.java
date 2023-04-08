package com.comino.mavmap.map.map3D.impl.octomap.boundingbox;

import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;

public class MAVBoundingBox extends OcTreeBoundingBoxWithCenterAndYaw {
	
	private final OcTreeSimpleBoundingBox box = new OcTreeSimpleBoundingBox();
	private final float resolution;
	
	public MAVBoundingBox(MAVOctoMap3D map) {
		super();
		this.resolution = map.getResolution();
	}
	
	
	public MAVBoundingBox(MAVOctoMap3D map,GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		super();
		this.resolution = map.getResolution();
		set(p,half_side_xy,half_side_z);
	}
	
	public MAVBoundingBox(MAVOctoMap3D map,GeoTuple4D_F32<?> p, float half_side_xy) {
		super();
		this.resolution = map.getResolution();
		setTop(p,half_side_xy);
	}
	
	public void set(GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		box.setMinX( p.x-half_side_xy); box.setMaxX( p.x+half_side_xy);
		box.setMinY( p.y-half_side_xy); box.setMaxY( p.y+half_side_xy);
		box.setMinZ(-p.z-half_side_z);  box.setMaxZ(-p.z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, 16);
	}
	
	public void setTop(GeoTuple4D_F32<?> p, float half_side_xyz) {
		box.setMinX( p.x-half_side_xyz); box.setMaxX( p.x+half_side_xyz);
		box.setMinY( p.y-half_side_xyz); box.setMaxY( p.y+half_side_xyz);
		box.setMinZ(-p.z-0.2f); box.setMaxZ(-p.z+half_side_xyz);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, 16);
	}


}
