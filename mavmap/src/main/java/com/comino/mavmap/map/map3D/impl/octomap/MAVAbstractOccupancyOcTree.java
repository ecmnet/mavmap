package com.comino.mavmap.map.map3D.impl.octomap;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.key.KeyRayReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeySet;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;
import us.ihmc.jOctoMap.occupancy.OccupancyParameters;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.rules.SetOccupancyRule;
import us.ihmc.jOctoMap.rules.UpdateOccupancyRule;
import us.ihmc.jOctoMap.rules.interfaces.CollidableRule;
import us.ihmc.jOctoMap.tools.OcTreeRayTools;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public abstract class MAVAbstractOccupancyOcTree<NODE extends AbstractOccupancyOcTreeNode<NODE>> extends AbstractOcTreeBase<NODE>
{
   // occupancy parameters of tree, stored in logodds:
   protected final OccupancyParameters occupancyParameters = new OccupancyParameters();
   /** Used to filter out points reducing the region of the OcTree to update. */
   protected OcTreeBoundingBoxInterface boundingBox;
   /**
    * Minimum range for how long individual beams are inserted (default -1: complete beam) when
    * inserting a ray or point cloud
    */
   protected double minInsertRange = -1.0;
   /**
    * Maximum range for how long individual beams are inserted (default -1: complete beam) when
    * inserting a ray or point cloud
    */
   protected double maxInsertRange = -1.0;
   /**
    * Discretize whether a scan to insert is discretized first into octree key cells (default: false).
    * This reduces the number of raycasts, resulting in a potential speedup.
    */
   private boolean discretizePointCloud = false;

   protected final UpdateOccupancyRule<NODE> updateOccupancyRule;
   protected final SetOccupancyRule<NODE> setOccupancyRule = new SetOccupancyRule<>();
   private final CollidableRule<NODE> collidableRule = new CollidableRule<NODE>()
   {
      @Override
      public boolean isCollidable(NODE node)
      {
         return isNodeOccupied(node);
      }
   };

   protected boolean useChangeDetection;
   /** Set of leaf keys (lowest level) which changed since last resetChangeDetection */
   protected final ConcurrentMap<OcTreeKeyReadOnly, Boolean> changedKeys = new ConcurrentHashMap<>();

   private final OcTreeKeySet freeCells = new OcTreeKeySet(1000000);
   private final OcTreeKeySet occupiedCells = new OcTreeKeySet(1000000);

   public MAVAbstractOccupancyOcTree(double resolution)
   {
      super(resolution);
      updateOccupancyRule = new UpdateOccupancyRule<>(occupancyParameters);
      useChangeDetection = false;
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected MAVAbstractOccupancyOcTree(double resolution, int treeDepth)
   {
      super(resolution, treeDepth);
      updateOccupancyRule = new UpdateOccupancyRule<>(occupancyParameters);
      useChangeDetection = false;
   }

   public MAVAbstractOccupancyOcTree(MAVAbstractOccupancyOcTree<NODE> other)
   {
      super(other);
      occupancyParameters.set(other.occupancyParameters);
      updateOccupancyRule = new UpdateOccupancyRule<>(occupancyParameters);
      boundingBox = other.boundingBox.getCopy();
      changedKeys.putAll(other.changedKeys);
      enableChangeDetection(other.useChangeDetection);
   }

   public void setOccupancyParameters(OccupancyParameters occupancyParameters)
   {
      this.occupancyParameters.set(occupancyParameters);
   }

   public OccupancyParametersReadOnly getOccupancyParameters()
   {
      return occupancyParameters;
   }

   /**
    * Minimum range for how long individual beams are inserted (default -1: complete beam) when
    * inserting a ray or point cloud
    */
   public void setMinimumInsertRange(double minRange)
   {
      minInsertRange = minRange;
   }

   /**
    * Maximum range for how long individual beams are inserted (default -1: complete beam) when
    * inserting a ray or point cloud
    */
   public void setMaximumInsertRange(double maxRange)
   {
      maxInsertRange = maxRange;
   }

   /**
    * Minimum and maximum range for how long individual beams are inserted (default -1: complete beam)
    * when inserting a ray or point cloud
    */
   public void setBoundsInsertRange(double minRange, double maxRange)
   {
      setMinimumInsertRange(minRange);
      setMaximumInsertRange(maxRange);
   }

   /** Remove the limitation in minimum range when inserting a ray or point cloud. */
   public void removeMinimumInsertRange()
   {
      minInsertRange = -1.0;
   }

   /** Remove the limitation in maximum range when inserting a ray or point cloud. */
   public void removeMaximumInsertRange()
   {
      maxInsertRange = -1.0;
   }

   /** Remove the limitation in minimum and maximum range when inserting a ray or point cloud. */
   public void removeBoundsInsertRange()
   {
      removeMinimumInsertRange();
      removeMaximumInsertRange();
   }

   /**
    * Discretize whether a scan to insert is discretized first into octree key cells (default: false).
    * This reduces the number of raycasts, resulting in a potential speedup.
    */
   public void enableDiscretizePointCloud(boolean enable)
   {
      discretizePointCloud = enable;
   }

   /**
    * Queries whether a node is occupied according to the tree's parameter for "occupancyThreshold"
    *
    * @param occupancyNode
    * @return
    */
   public boolean isNodeOccupied(NODE occupancyNode)
   {
      return OccupancyTools.isNodeOccupied(occupancyParameters, occupancyNode);
   }

   public void insertSweepCollection(ScanCollection scanCollection)
   {
      freeCells.clear();
      occupiedCells.clear();

      for (int i = 0; i < scanCollection.getNumberOfScans(); i++)
      {
         Scan scan = scanCollection.getScan(i);
         PointCloud pointCloud = scan.getPointCloud();
         Point3DReadOnly sensorOrigin = scan.getSensorOrigin();

         if (discretizePointCloud)
            OcTreeRayTools.computeDiscreteUpdate(sensorOrigin,
                                                 pointCloud,
                                                 freeCells,
                                                 occupiedCells,
                                                 boundingBox,
                                                 minInsertRange,
                                                 maxInsertRange,
                                                 resolution,
                                                 treeDepth);
         else
            OcTreeRayTools.computeUpdate(sensorOrigin,
                                         pointCloud,
                                         freeCells,
                                         occupiedCells,
                                         boundingBox,
                                         minInsertRange,
                                         maxInsertRange,
                                         resolution,
                                         treeDepth);
      }

      // insert data into tree  -----------------------
      for (OcTreeKeyReadOnly key : occupiedCells)
         updateNode(key, true);

      for (OcTreeKeyReadOnly key : freeCells)
         updateNode(key, false);
   }

   /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP. Special care is
    * taken that each voxel in the map is updated only once, and occupied nodes have a preference over
    * free ones. This avoids holes in the floor from mutual deletion and is more efficient than the
    * plain ray insertion in insertPointCloudRays().
    *
    * @note replaces insertScan()
    * @param scan         Pointcloud (measurement endpoints), in global reference frame
    * @param sensorOrigin measurement origin in global reference frame
    */
   public void insertPointCloud(PointCloud scan, Point3DReadOnly sensorOrigin)
   {
      freeCells.clear();
      occupiedCells.clear();

      if (discretizePointCloud)
         OcTreeRayTools.computeDiscreteUpdate(sensorOrigin, scan, freeCells, occupiedCells, boundingBox, minInsertRange, maxInsertRange, resolution, treeDepth);
      else
         OcTreeRayTools.computeUpdate(sensorOrigin, scan, freeCells, occupiedCells, boundingBox, minInsertRange, maxInsertRange, resolution, treeDepth);

      // insert data into tree  -----------------------
      for (OcTreeKeyReadOnly key : occupiedCells)
         updateNode(key, true);

      for (OcTreeKeyReadOnly key : freeCells)
         updateNode(key, false);
   }

   /**
    * Integrate a 3d scan (transform scan before tree update), parallelized with OpenMP. Special care
    * is taken that each voxel in the map is updated only once, and occupied nodes have a preference
    * over free ones. This avoids holes in the floor from mutual deletion and is more efficient than
    * the plain ray insertion in insertPointCloudRays().
    *
    * @note replaces insertScan()
    * @param scan         Pointcloud (measurement endpoints) relative to frame origin
    * @param sensorOrigin origin of sensor relative to frame origin
    * @param frameOrigin  origin of reference frame, determines transform to be applied to cloud and
    *                     sensor origin
    */
   public void insertPointCloud(PointCloud scan, Point3DReadOnly sensorOrigin, Transform frameOrigin)
   {
      // performs transformation to data and sensor origin first
      PointCloud transformedScan = new PointCloud(scan);
      transformedScan.transform(frameOrigin);
      Point3D transformedSensorOrigin = new Point3D(sensorOrigin);
      frameOrigin.transform(transformedSensorOrigin);
      insertPointCloud(transformedScan, transformedSensorOrigin);
   }

   /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP. This function
    * simply inserts all rays of the point clouds as batch operation. Discretization effects can lead
    * to the deletion of occupied space, it is usually recommended to use insertPointCloud() instead.
    *
    * @param scan         Pointcloud (measurement endpoints), in global reference frame
    * @param sensorOrigin measurement origin in global reference frame
    */
   public void insertPointCloudRays(PointCloud scan, Point3DReadOnly sensorOrigin)
   {
      if (scan.getNumberOfPoints() < 1)
         return;

      Vector3D direction = new Vector3D();

      for (int i = 0; i < scan.getNumberOfPoints(); i++)
      {
         Point3D point = new Point3D(scan.getPoint(i));
         direction.sub(point, sensorOrigin);
         double length = direction.length();
         if (minInsertRange > 0.0 && length < minInsertRange)
            continue;

         if (maxInsertRange > 0.0 && length > maxInsertRange)
         {
            point.scaleAdd(maxInsertRange / length, direction, sensorOrigin);
            KeyRayReadOnly ray = OcTreeRayTools.computeRayKeys(sensorOrigin, point, resolution, treeDepth);
            if (ray != null)
            {
               for (int j = 0; j < ray.size(); j++)
                  updateNode(ray.get(j), false); // insert freespace measurement
            }
         }
         else
         {
            KeyRayReadOnly ray = OcTreeRayTools.computeRayKeys(sensorOrigin, point, resolution, treeDepth);
            if (ray != null)
            {
               for (int j = 0; j < ray.size(); j++)
                  updateNode(ray.get(j), false); // insert freespace measurement
               updateNode(point, true); // update endpoint to be occupied
            }
         }
      }
   }

   /**
    * Set log_odds value of voxel to logOddsValue. This only works if key is at the lowest octree level
    *
    * @param key          OcTreeKey of the NODE that is to be updated
    * @param logOddsValue value to be set as the log_odds value of the node
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(OcTreeKeyReadOnly key, float logOddsValue)
   {
      // clamp log odds within range:
      setOccupancyRule.setNewLogOdds(OccupancyTools.clipLogOddsToMinMax(occupancyParameters, logOddsValue));
      return updateNodeInternal(key, setOccupancyRule, null);
   }

   /**
    * Set log_odds value of voxel to logOddsValue. Looks up the OcTreeKey corresponding to the
    * coordinate and then calls setNodeValue() with it.
    *
    * @param coordinate   3d coordinate of the NODE that is to be updated
    * @param logOddsValue value to be set as the log_odds value of the node
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(Point3DReadOnly coordinate, float logOddsValue)
   {
      return setNodeValue(coordinate.getX(), coordinate.getY(), coordinate.getZ(), logOddsValue);
   }

   /**
    * Set log_odds value of voxel to logOddsValue. Looks up the OcTreeKey corresponding to the
    * coordinate and then calls setNodeValue() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param logOddsValue value to be set as the log_odds value of the node
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(double x, double y, double z, float logOddsValue)
   {
      // clamp log odds within range:
      setOccupancyRule.setNewLogOdds(OccupancyTools.clipLogOddsToMinMax(occupancyParameters, logOddsValue));
      return updateNodeInternal(x, y, z, setOccupancyRule, null);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative). This only works
    * if key is at the lowest octree level
    *
    * @param key           OcTreeKey of the NODE that is to be updated
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKeyReadOnly key, float logOddsUpdate)
   {
      updateOccupancyRule.setUpdateLogOdds(logOddsUpdate);
      return updateNodeInternal(key, updateOccupancyRule, updateOccupancyRule);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative). Looks up the
    * OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param coordinate    3d coordinate of the NODE that is to be updated
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3DReadOnly coordinate, float logOddsUpdate)
   {
      updateOccupancyRule.setUpdateLogOdds(logOddsUpdate);
      return updateNodeInternal(coordinate, updateOccupancyRule, updateOccupancyRule);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative). Looks up the
    * OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, float logOddsUpdate)
   {
      updateOccupancyRule.setUpdateLogOdds(logOddsUpdate);
      return updateNodeInternal(x, y, z, updateOccupancyRule, updateOccupancyRule);
   }

   /**
    * Integrate occupancy measurement.
    *
    * @param key      OcTreeKey of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKeyReadOnly key, boolean occupied)
   {
      return updateNode(key, occupancyParameters.getUpdateLogOdds(occupied));
   }

   /**
    * Integrate occupancy measurement. Looks up the OcTreeKey corresponding to the coordinate and then
    * calls udpateNode() with it.
    *
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param occupied   true if the node was measured occupied, else false
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3DReadOnly coordinate, boolean occupied)
   {
      return updateNode(coordinate.getX(), coordinate.getY(), coordinate.getZ(), occupied);
   }

   /**
    * Integrate occupancy measurement. Looks up the OcTreeKey corresponding to the coordinate and then
    * calls udpateNode() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param occupied true if the node was measured occupied, else false
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, boolean occupied)
   {
      OcTreeKey key = coordinateToKey(x, y, z);
      if (key == null)
         return null;
      return updateNode(key, occupied);
   }

   /**
    * Creates the maximum likelihood map by calling toMaxLikelihood on all tree nodes, setting their
    * occupancy to the corresponding occupancy thresholds. This enables a very efficient compression if
    * you call prune() afterwards.
    */
   public void toMaxLikelihood()
   {
      if (root == null)
         return;

      // convert bottom up
      for (int depth = treeDepth; depth > 0; depth--)
      {
         toMaxLikelihoodRecurs(root, 0, depth);
      }

      // convert root
      OccupancyTools.nodeToMaxLikelihood(occupancyParameters, root);
   }

   /**
    * Insert one ray between origin and end into the tree. integrateMissOnRay() is called for the ray,
    * the end point is updated as occupied. It is usually more efficient to insert complete pointclouds
    * with insertPointCloud() or insertPointCloudRays().
    *
    * @param origin         origin of sensor in global coordinates
    * @param end            endpoint of measurement in global coordinates
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *                       This speeds up the insertion, but you need to call updateInnerOccupancy()
    *                       when done.
    * @return success of operation
    */
   public boolean insertRay(Point3DReadOnly origin, Point3DReadOnly end)
   {
      Vector3D direction = new Vector3D();
      direction.sub(end, origin);
      double length = direction.length();

      if (minInsertRange > 0.0 && length < minInsertRange)
         return false;

      // cut ray at maxrange
      if (maxInsertRange > 0 && length > maxInsertRange)
      {
         direction.scale(1.0 / length);
         Point3D newEnd = new Point3D();
         newEnd.scaleAdd(maxInsertRange, direction, origin);
         return integrateMissOnRay(origin, newEnd);
      }
      // insert complete ray
      else
      {
         if (!integrateMissOnRay(origin, end))
            return false;
         updateNode(end, true); // insert hit cell
         return true;
      }
   }

   public boolean castRay(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DBasics endToPack)
   {
      return castRay(origin, direction, endToPack, false);
   }

   public boolean castRay(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DBasics endToPack, boolean ignoreUnknownCells)
   {
      return castRay(origin, direction, endToPack, ignoreUnknownCells, -1.0);
   }

   /**
    * Performs raycasting in 3d, similar to computeRay(). Can be called in parallel e.g. with OpenMP
    * for a speedup. A ray is cast from 'origin' with a given direction, the first non-free cell is
    * returned in 'end' (as center coordinate). This could also be the origin node if it is occupied or
    * unknown. castRay() returns true if an occupied node was hit by the raycast. If the raycast
    * returns false you can search() the node at 'end' and see whether it's unknown space.
    *
    * @param[in] origin starting coordinate of ray
    * @param[in] direction A vector pointing in the direction of the raycast (NOT a point in space).
    *            Does not need to be normalized.
    * @param[out] endToPack returns the center of the last cell on the ray. If the function returns
    *             true, it is occupied.
    * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false
    *            (default), the raycast aborts when an unknown cell is hit and returns false.
    * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
    * @return true if an occupied cell was hit, false if the maximum range or octree bounds are
    *         reached, or if an unknown node was hit.
    */
   public boolean castRay(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DBasics endToPack, boolean ignoreUnknownCells, double maxRange)
   {
      return OcTreeRayTools.castRay(root, origin, direction, endToPack, ignoreUnknownCells, maxRange, collidableRule, resolution, treeDepth);
   }

   public boolean getRayIntersection(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DReadOnly center, Point3DBasics intersection)
   {
      return getRayIntersection(origin, direction, center, intersection, 0.0);
   }

   /**
    * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the
    * ray originating from origin and a plane of the axis aligned cube.
    *
    * @param[in] origin Starting point of ray
    * @param[in] direction A vector pointing in the direction of the raycast. Does not need to be
    *            normalized.
    * @param[in] center The center of the voxel where the ray terminated. This is the output of
    *            castRay.
    * @param[out] intersectionToPack The entry point of the ray into the voxel, on the voxel surface.
    * @param[in] delta A small increment to avoid ambiguity of being exactly on a voxel surface. A
    *            positive value will get the point out of the hit voxel, while a negative value will
    *            get it inside.
    * @return Whether or not an intesection point has been found. Either, the ray never cross the voxel
    *         or the ray is exactly parallel to the only surface it intersect.
    */
   public boolean getRayIntersection(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DReadOnly center, Point3DBasics intersectionToPack, double delta)
   {
      return OcTreeRayTools.getRayIntersection(origin, direction, center, intersectionToPack, delta, resolution);
   }

   public void disableBoundingBox()
   {
      boundingBox = null;
   }

   /**
    * Bounding box to use for the next updates on this OcTree. If null, no limit will be applied.
    *
    * @param boundingBox
    */
   public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
   {
      this.boundingBox = boundingBox;
   }

   public OcTreeBoundingBoxInterface getBoundingBox()
   {
      return boundingBox;
   }

   /**
    * @return true if point is in the currently set bounding box or if there is no bounding box.
    */
   public boolean isInBoundingBox(Point3DReadOnly candidate)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(candidate);
   }

   /**
    * @return true if key is in the currently set bounding box or if there is no bounding box.
    */
   public boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(candidate);
   }

   public void enableChangeDetection(boolean enable)
   {
      useChangeDetection = enable;
      if (useChangeDetection)
         updateOccupancyRule.attachChangedKeys(changedKeys);
      else
         updateOccupancyRule.detachChangedKeys();
   }

   public boolean isChangeDetectionEnabled()
   {
      return useChangeDetection;
   }

   public void resetChangeDetection()
   {
      changedKeys.clear();
   }

   public int numberOfChangesDetected()
   {
      return changedKeys.size();
   }

   public Map<OcTreeKeyReadOnly, Boolean> getChangedKeys()
   {
      return changedKeys;
   }

   /**
    * Updates the occupancy of all inner nodes to reflect their children's occupancy. If you performed
    * batch-updates with lazy evaluation enabled, you must call this before any queries to ensure
    * correct multi-resolution behavior.
    **/
   public void updateInnerOccupancy()
   {
      if (root != null)
         updateInnerOccupancyRecurs(root, 0);
   }

   /**
    * Integrate a "hit" measurement according to the tree's sensor model
    *
    * @param occupancyNode
    */
   public void integrateHit(NODE occupancyNode)
   {
      OccupancyTools.updateNodeLogOdds(occupancyParameters, occupancyNode, occupancyParameters.getHitProbabilityLogOdds());
   }

   /**
    * Integrate a "miss" measurement according to the tree's sensor model
    *
    * @param occupancyNode
    */
   public void integrateMiss(NODE occupancyNode)
   {
      OccupancyTools.updateNodeLogOdds(occupancyParameters, occupancyNode, occupancyParameters.getMissProbabilityLogOdds());
   }

   /**
    * Update logodds value of node by adding to the current value.
    *
    * @param occupancyNode
    * @param update
    */
   public void updateNodeLogOdds(NODE occupancyNode, float update)
   {
      OccupancyTools.updateNodeLogOdds(occupancyParameters, occupancyNode, update);
   }

   /**
    * Traces a ray from origin to end and updates all voxels on the way as free. The volume containing
    * "end" is not updated.
    */
   protected boolean integrateMissOnRay(Point3DReadOnly origin, Point3DReadOnly end)
   {
      KeyRayReadOnly ray = OcTreeRayTools.computeRayKeys(origin, end, resolution, treeDepth);

      if (ray == null)
         return false;

      for (int i = 0; i < ray.size(); i++)
         updateNode(ray.get(i), false); // insert freespace measurement

      return true;
   }

   protected void updateInnerOccupancyRecurs(NODE node, int depth)
   {
      if (node == null)
         throw new RuntimeException("The given node is null.");

      // only recurse and update for inner nodes:
      if (node.hasAtLeastOneChild())
      {
         // return early for last level:
         if (depth < treeDepth)
         {
            for (int i = 0; i < 8; i++)
            {
               NODE childNode = node.getChild(i);
               if (childNode != null)
                  updateInnerOccupancyRecurs(childNode, depth + 1);
            }
         }
         node.updateOccupancyChildren();
      }
   }

   protected void toMaxLikelihoodRecurs(NODE node, int depth, int max_depth)
   {
      if (node == null)
         throw new RuntimeException("The given node is null.");

      if (depth < max_depth)
      {
         for (int i = 0; i < 8; i++)
         {
            NODE childNode = node.getChild(i);
            if (childNode != null)
               toMaxLikelihoodRecurs(childNode, depth + 1, max_depth);
         }
      }
      else
      { // max level reached
         OccupancyTools.nodeToMaxLikelihood(occupancyParameters, node);
      }
   }
}
