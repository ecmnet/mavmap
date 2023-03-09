package com.comino.mavmap.map.map3D.impl.octomap;


import java.util.Map;

import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;

public class MAVOccupancyOcTree extends AbstractOccupancyOcTree<MAVOccupancyOcTreeNode>
{
   public MAVOccupancyOcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected Class<MAVOccupancyOcTreeNode> getNodeClass()
   {
      return MAVOccupancyOcTreeNode.class;
   }
   
   public Map<OcTreeKeyReadOnly, Boolean> getChangedMap() {
	   return changedKeys;
   }
   
  
}
