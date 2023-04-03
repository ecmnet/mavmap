package com.comino.mavmap.map.map3D.impl.octomap;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;

public final class MAVOccupancyOcTreeNode extends AbstractOccupancyOcTreeNode<MAVOccupancyOcTreeNode> {

	long tms = 0;

	public MAVOccupancyOcTreeNode() {
		super();
		this.tms = System.nanoTime();
	}
	
	public void setValidityToInfinite() {
		this.tms = Long.MAX_VALUE;
	}
	
	public void outdate() {
		this.tms = 0;
		super.setLogOdds(1.0e-7f);
	}
	
	@Override
	public void resetLogOdds() {
		this.tms = System.nanoTime();
	  super.resetLogOdds();
	}

	@Override
	public void clear() {
	this.resetLogOdds();
	}

	@Override
	public void setLogOdds(float f) {
		this.tms = System.nanoTime();
		super.setLogOdds(f);
	}

	@Override
	public void addValue(float logOdds) {
		this.tms = System.nanoTime();
		super.addValue(logOdds);
	}

	public long getTimestamp() {
		return tms;
	}
	
	public void getCenter(GeoTuple3D_F32<?> p) {
		p.x =  (float)this.getX();
		p.y =  (float)this.getY();
		p.z = -(float)this.getZ();
	}

}
