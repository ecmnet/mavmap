package com.comino.mavmap.map.map3D.impl.octomap.rule;

import java.util.Map;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.EarlyAbortRule;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class MAVOccupancyUpdateRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>, EarlyAbortRule<NODE> {
	
	   public final static byte  CREATED = 0;
	   public final static byte  UPDATED = 1;
	   public final static byte  DELETED = 2;

	   private float updateLogOdds = Float.NaN;
	   private final OccupancyParametersReadOnly parameters;
	   private Map<OcTreeKeyReadOnly, Byte> changedKeys;
	
	   public MAVOccupancyUpdateRule(OccupancyParametersReadOnly occupancyParameters)
	   {
	      this.parameters = occupancyParameters;
	   }

	   public void setUpdateLogOdds(float updateLogOdds)
	   {
	      this.updateLogOdds = updateLogOdds;
	   }

	   public void detachChangedKeys()
	   {
	      changedKeys = null;
	   }

	   public void attachChangedKeys(Map<OcTreeKeyReadOnly, Byte> changedKeys)
	   {
	      this.changedKeys = changedKeys;
	   }

	@Override
	public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated) {
		if (changedKeys != null)
	      {
	         boolean occupiedBefore = OccupancyTools.isNodeOccupied(parameters, leafToUpdate);
	         OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

	         if (nodeJustCreated)
	         { // new node
	            changedKeys.put(leafKey, CREATED);
	         }
	         else if (occupiedBefore != OccupancyTools.isNodeOccupied(parameters, leafToUpdate))
	         { // occupancy changed, track it
	            Byte changedKeyValue = changedKeys.get(leafKey);
	            if (changedKeyValue == null)
	               changedKeys.put(leafKey, UPDATED);
	            else if (changedKeyValue == UPDATED)
	               changedKeys.remove(leafKey);
	         }
	      }
	      else
	      {
	         OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);
	      }
		
	}

	@Override
	public void updateInnerNode(NODE innerNodeToUpdate) {
		  innerNodeToUpdate.updateOccupancyChildren();
	}
	
	@Override
	public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate) {
		if (nodeToUpdate == null)
	         return false;

	      // early abort (no change will happen).
	      // may cause an overhead in some configuration, but more often helps
	      // no change: node already at threshold
	      float nodeLogOdds = nodeToUpdate.getLogOdds();
	      boolean reachedMaxThreshold = updateLogOdds >= 0.0f && nodeLogOdds >= parameters.getMaxLogOdds();
	      boolean reachedMinThreshold = updateLogOdds <= 0.0f && nodeLogOdds <= parameters.getMinLogOdds();
	      return reachedMaxThreshold || reachedMinThreshold;
	}

}
