package com.comino.mavmap.map.map3D.impl.octomap;


import java.util.Map;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;

public class MAVOccupancyOcTree extends MAVAbstractOccupancyOcTree<MAVOccupancyOcTreeNode>
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
   
  
}
