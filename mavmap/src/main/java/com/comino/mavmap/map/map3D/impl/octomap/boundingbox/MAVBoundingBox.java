package com.comino.mavmap.map.map3D.impl.octomap.boundingbox;

import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;

import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;

public class MAVBoundingBox extends OcTreeBoundingBoxWithCenterAndYaw {
	
	private final OcTreeSimpleBoundingBox box = new OcTreeSimpleBoundingBox();
	private final float resolution;
	private final int   treeDepth;
	
	public MAVBoundingBox(MAVOccupancyOcTree map) {
		super();
		this.resolution = (float)map.getResolution();
		this.treeDepth  = map.getTreeDepth();
	}
	public MAVBoundingBox(MAVOccupancyOcTree map,GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		this(map);
		set(p,half_side_xy,half_side_z);
	}
	
	public MAVBoundingBox(MAVOccupancyOcTree map,GeoTuple4D_F32<?> p, float half_side_xy) {
		this(map);
		setTop(p,half_side_xy);
	}
	
	public void set(GeoTuple4D_F32<?> p, float half_side_xy, float half_side_z) {
		box.setMinX( p.x-half_side_xy); box.setMaxX( p.x+half_side_xy);
		box.setMinY( p.y-half_side_xy); box.setMaxY( p.y+half_side_xy);
		box.setMinZ(-p.z-half_side_z);  box.setMaxZ(-p.z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, treeDepth);
	}
	
	public void setTop(GeoTuple4D_F32<?> p, float half_side_xyz) {
		box.setMinX( p.x-half_side_xyz); box.setMaxX( p.x+half_side_xyz);
		box.setMinY( p.y-half_side_xyz); box.setMaxY( p.y+half_side_xyz);
		box.setMinZ(-p.z-0.2f); box.setMaxZ(-p.z+half_side_xyz);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, treeDepth);
	}
	
	public void set(GeoTuple4D_F32<?> p, float half_side_x, float half_side_y, float half_side_z) {
		box.setMinX( p.x-half_side_x); box.setMaxX( p.x+half_side_x);
		box.setMinY( p.y-half_side_y); box.setMaxY( p.y+half_side_y);
		box.setMinZ( p.z-half_side_z); box.setMaxZ (p.z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, treeDepth);
	}
	
	public void set(float x, float y, float z, float half_side_x, float half_side_y, float half_side_z) {
		box.setMinX( x-half_side_x); box.setMaxX( x+half_side_x);
		box.setMinY( y-half_side_y); box.setMaxY( y+half_side_y);
		box.setMinZ( z-half_side_z); box.setMaxZ (z+half_side_z);
		
		this.setLocalBoundingBox(box);
		this.setYaw(0);
		this.update(this.resolution, treeDepth);
	}


}
