package com.comino.mavmap.map.map3D.impl.octomap.node;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.key.KeyRay;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeySet;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeRayTools;

public abstract class MAVOcTreeRayTools {


	public static <NODE extends AbstractOcTreeNode<NODE>> void computeDiscreteUpdate(Point3DReadOnly origin, PointCloud pointCloud, OcTreeKeySet freeCells,
			OcTreeKeySet occupiedCells, OcTreeBoundingBoxInterface boundingBox,
			double minRange, double maxRange, double resolution, int treeDepth)
	{
		PointCloud discretePC = new PointCloud();
		OcTreeKeySet endpoints = new OcTreeKeySet();

		for (int i = 0; i < pointCloud.getNumberOfPoints(); ++i)
		{
			OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(pointCloud.getPoint(i), resolution, treeDepth);
			if (endpoints.add(key)) // insertion took place => key was not in set
				discretePC.add(OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth));
		}

		computeUpdate(origin, discretePC, freeCells, occupiedCells, boundingBox, minRange, maxRange, resolution, treeDepth);
	}


	public static <NODE extends AbstractOcTreeNode<NODE>> void computeUpdate(Point3DReadOnly origin, PointCloud pointCloud, OcTreeKeySet freeCells,
			OcTreeKeySet occupiedCells, OcTreeBoundingBoxInterface boundingBox, double minRange,
			double maxRange, double resolution, int treeDepth)
	{
		OcTreeKeySet unfilteredFreeCells = new OcTreeKeySet();
	//	OcTreeKey key = new OcTreeKey();
		Vector3D direction = new Vector3D();
		Point3D point = new Point3D();

		for (int i = 0; i < pointCloud.getNumberOfPoints(); ++i)
		{
			OcTreeKey key = new OcTreeKey();
			point.set(pointCloud.getPoint(i));
			direction.sub(point, origin);
			double length = direction.norm();

			if (minRange >= 0.0 && length < minRange)
				continue;

			if (boundingBox == null)
			{ // no BBX specified
				if (maxRange < 0.0 || length <= maxRange)
				{ // is not maxrange meas.
					// free cells
					KeyRay ray = OcTreeRayTools.computeRayKeys(origin, point, resolution, treeDepth);
					if (ray != null) {
						for(int j=0;j < ray.size();j++)
							unfilteredFreeCells.add(ray.get(j));
						//	unfilteredFreeCells.addAll(ray);
					}
					// occupied endpoint
					if (OcTreeKeyConversionTools.coordinateToKey(point, resolution, treeDepth, key))
						occupiedCells.add(key);
				}
				else
				{ // user set a maxrange and length is above
					Point3D newEnd = new Point3D();
					newEnd.scaleAdd(maxRange / length, direction, origin);
					KeyRay ray = OcTreeRayTools.computeRayKeys(origin, newEnd, resolution, treeDepth);
					if (ray != null)
						for(int j=0;j < ray.size();j++)
							unfilteredFreeCells.add(ray.get(j));
				} // end if maxrange
			}
			else
			{ // BBX was set
				// endpoint in bbx and not maxrange?
				if (boundingBox.isInBoundingBox(point) && (maxRange < 0.0 || length <= maxRange))
				{
					// occupied endpoint
					if (OcTreeKeyConversionTools.coordinateToKey(point, resolution, treeDepth, key))
						occupiedCells.add(key);

					// update freespace, break as soon as bbx limit is reached
					KeyRay ray = OcTreeRayTools.computeRayKeys(origin, point, resolution, treeDepth);
					if (ray != null)
					{
						for (int j = ray.size() - 1; j >= 0; j--)
						{
							OcTreeKeyReadOnly currentKey = ray.get(j);
							if (boundingBox.isInBoundingBox(currentKey))
								unfilteredFreeCells.add(currentKey);
							else
								break;
						}
					} // end if compute ray
				} // end if in BBX and not maxrange
			} // end bbx case

		} // end for all points, end of parallel OMP loop

		// prefer occupied cells over free ones (and make sets disjunct)
		for (OcTreeKeyReadOnly possibleFreeCell : unfilteredFreeCells)
		{
			if (!occupiedCells.contains(possibleFreeCell))
				freeCells.add(possibleFreeCell);
		}
	}

}
