package com.comino.mavmap.map.map3D.impl.octomap;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;

public final class MAVOccupancyOcTreeNode extends AbstractOccupancyOcTreeNode<MAVOccupancyOcTreeNode> {

	long tms = 0;

	public MAVOccupancyOcTreeNode() {
		super();
		this.tms = System.currentTimeMillis();
	}

	@Override
	public void clear() {
	  super.resetLogOdds();
	}

	@Override
	public void setLogOdds(float f) {
		this.tms = System.currentTimeMillis();
		super.setLogOdds(f);
	}

	public long getTimestamp() {
		return tms;
	}



}
