package com.comino.mavmap.map.map3D.impl.octomap;

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
