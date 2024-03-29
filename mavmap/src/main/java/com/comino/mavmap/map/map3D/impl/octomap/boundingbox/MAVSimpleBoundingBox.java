package com.comino.mavmap.map.map3D.impl.octomap.boundingbox;

import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;

public class MAVSimpleBoundingBox extends OcTreeSimpleBoundingBox {

	private final float resolution;
	private final int   depth;
	private final GeoTuple4D_F32<?> p;


	public MAVSimpleBoundingBox(float resolution, int depth ) {
		super();
		this.p          = new Point4D_F32();
		this.resolution = resolution;
		this.depth      = depth;
	}

	public void set(GeoTuple4D_F32<?> p, float length) {

		float l2 = length / 2.0f;

		this.setMinX( p.x-l2); this.setMaxX( p.x+l2);
		this.setMinY( p.y-l2); this.setMaxY( p.y+l2);
		this.setMinZ(-p.z-l2); this.setMaxZ(-p.z+l2);

		this.update(resolution, depth);

	}
	
	public GeoTuple4D_F32<?> getCenter() {
		return p;
	}

	public void set(GeoTuple4D_F32<?> p, float length, float height) {
		
		this.p.setTo(p.x,p.y,p.z,p.w);

		float l2 = length / 2.0f;
		float h2 = height / 2.0f;
		
		this.setMinMaxCoordinates( this.p.x-l2, this.p.y-l2, -this.p.z-h2, this.p.x+l2, this.p.y+l2, -this.p.z+h2);
		this.update(resolution, depth);

	}
	
}
