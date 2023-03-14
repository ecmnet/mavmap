package com.comino.mavmap.map.map3D.impl.octomap;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import com.comino.mavmap.map.map3D.impl.octomap.boundingbox.MAVBoundingBox;

import georegression.struct.GeoTuple3D_F32;
import georegression.struct.GeoTuple4D_F32;
import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector3D_F32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class MAVOctoMap3D {

	public final static double  INDETERMINED = -1;
	public final static int     DELETED      =  4;

	private final static double RESOLUTION   = 0.2;

	private MAVOccupancyOcTree         map;
	private final Point3D              tmp;
	private final OcTreeKey            tmpkey;

	private final List<Long>           deletedNodeListenCoded;
	private final List<Long> encodedList = new ArrayList<Long>();
	private boolean allowRemoveOutDated;

	public MAVOctoMap3D() {
		this(RESOLUTION,true);
	}

	public MAVOctoMap3D(double resolution, boolean allowRemoveOutDated) {

		this.map    = new MAVOccupancyOcTree(resolution);
		this.map.enableChangeDetection(true);
		this.tmp    = new Point3D();
		this.tmpkey = new OcTreeKey();

		this.deletedNodeListenCoded = new ArrayList<Long>();

		this.allowRemoveOutDated = allowRemoveOutDated;

	}

	public void disableRemoveOutdated() {
		this.allowRemoveOutDated = false;
	}
	
	public void enableRemoveOutdated() {
		this.allowRemoveOutDated = true;
	}

	public void update(GeoTuple3D_F32<?> p, GeoTuple3D_F32<?> o ) {
		Point3D origin = new Point3D(p.x,p.y,-p.z);
		Point3D end = new Point3D(o.x,o.y,-o.z);
		map.insertRay(origin, end);		
	}

	public MAVOccupancyOcTree getTree() {
		return map;
	}

	public void insert(GeoTuple3D_F32<?> o) {
		OcTreeKey key = map.coordinateToKey(o.x, o.y, -o.z);
		map.updateNode(key,true);
	}

	public void insert(GeoTuple4D_F32<?> o) {
		OcTreeKey key = map.coordinateToKey(o.x, o.y, -o.z);
		map.updateNode(key,o.w);
	}

	public void insert(float x, float y, float z) {
		OcTreeKey key = map.coordinateToKey(x, y, z);
		map.updateNode(key,true);
	}


	public double search(float x, float y, float z) {
		tmp.set(x,y,-z);
		MAVOccupancyOcTreeNode node = map.search(tmp);
		if(node!=null)
			return node.getOccupancy();	
		return INDETERMINED;
	}

	public double search(GeoTuple3D_F32<?> p) {
		return search(p.x,p.y,p.z);
	}

	public void clear() {
		// TODO Better to clear existing Tree
		this.map = new MAVOccupancyOcTree(RESOLUTION);
		this.map.enableChangeDetection(true);
	}

	public float getResolution() {
		return (float)map.getResolution();
	}

	public long convertTo(OcTreeKeyReadOnly key, GeoTuple4D_F32<?> target) {
		MAVOccupancyOcTreeNode node = map.search(key);
		if(node==null)
			return 0;
		target.setTo((float)node.getX(), (float)node.getY(), (float)node.getZ(), node.getLogOdds());
		return encode(key,1);
	}

	public Set<OcTreeKeyReadOnly> getChanged() {
		Set<OcTreeKeyReadOnly> keys = map.getChangedKeys().keySet();
		return keys;
	}
	
	public OcTreeIterable<MAVOccupancyOcTreeNode> getLeafsAtPosition(  GeoTuple4D_F32<?> p, float l, float h) {
		return OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), new MAVBoundingBox(p,l, h));
	}
	

	public List<Long> getLeafsAtPositionEncoded(  GeoTuple4D_F32<?> p, float l, float h) {
		encodedList.clear();
		
		OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), new MAVBoundingBox(p,l, h)).
		   forEach((n) ->  {
			 if(OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), n))
				 encodedList.add(encode(n,1));
	    });

		return encodedList;	
	}


	public int getNumberOfNodes() {
		return map.getNumberOfLeafNodes();
	}

	public void resetChangeDetection() {
		map.resetChangeDetection();
	}

	public List<Long> getDeletedEncoded() {
		return deletedNodeListenCoded;
	}

	public void clearDeletedEncoded() {
		deletedNodeListenCoded.clear();
	}

	public long countOccupiedNodes() {
		OcTreeIterable<MAVOccupancyOcTreeNode> leaf_nodes = OcTreeIteratorFactory.createLeafIterable(map.getRoot());
		return leaf_nodes.toList().parallelStream().
				filter(n -> OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), n)).count();
	}


	public List<Long> getChangedEncoded(boolean reset) {

		List<Long> encodedList = new ArrayList<Long>();
		encodedList.clear();

		if(map.numberOfChangesDetected()==0) {
			return encodedList;
		}

		Set<OcTreeKeyReadOnly> keys = map.getChangedKeys().keySet();

		for(OcTreeKeyReadOnly key : keys) {
			//	System.out.println(key);
			MAVOccupancyOcTreeNode node = map.search(key);
			if(node==null)
				continue;
			if(OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), node))
				encodedList.add(encode(key,1));
			else
				encodedList.add(encode(key,0));
		}
		if(reset)
			map.resetChangeDetection();

		return encodedList;	
	}


	public void removeOutdatedNodes(long dt_ms) {

		if(!allowRemoveOutDated)
			return;

		long tms = System.currentTimeMillis()-dt_ms;

		OcTreeIterable<MAVOccupancyOcTreeNode> leaf_nodes = OcTreeIteratorFactory.createLeafIterable(map.getRoot());

		leaf_nodes.toList().parallelStream().forEach((node) -> {
			if(node.getTimestamp() < tms) {
				OcTreeKey key = new OcTreeKey();
				node.getKey(key);
				node.setLogOdds(1e-7f);
				node.tms = Long.MAX_VALUE;
				map.getChangedKeys().put(key, false);
			}
		});

		map.prune();
	}

	public void invalidateAllOccupied() {

		Iterator<MAVOccupancyOcTreeNode> all_nodes = map.iterator();
		while(all_nodes.hasNext()) {
			MAVOccupancyOcTreeNode node = all_nodes.next();
			if(OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), node))
				map.getChangedKeys().put(node.getKeyCopy(), true);	
		}	
	}

	public List<Long> getTreeEncoded() {
		List<Long> encodedList = new ArrayList<Long>();
		encodedList.clear();
		if(map.getNumberOfNodes()==0)
			return encodedList;

		Iterator<MAVOccupancyOcTreeNode> all_nodes = map.iterator();

		while(all_nodes.hasNext()) {
			MAVOccupancyOcTreeNode node = all_nodes.next();
			if(OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), node))
				encodedList.add(encode(node.getKey0(), node.getKey1(), node.getKey2(),1));
			else
				encodedList.add(encode(node.getKey0(), node.getKey1(), node.getKey2(),0));
		}
		return encodedList;	
	}
	
	public long encode(MAVOccupancyOcTreeNode k, int value_4bit) {
		return encode(k.getKey0(), k.getKey1(), k.getKey2(),value_4bit);
	}

	public long encode(OcTreeKeyReadOnly k, int value_4bit) {
		return encode(k.getKey(0), k.getKey(1), k.getKey(2),value_4bit);
	}

	public int decode(long encoded, OcTreeKey target) {
		target.set((int)((encoded >> 40) & 0xFFFFFL), (int)((encoded >> 20) & 0xFFFFFL), (int)(encoded & 0xFFFFFL));
		return ((int)((encoded >> 60) & 0xFFFFL));
	}

	private long encode(int k0, int k1, int k2, int value_4bit) {
		return (((long)(value_4bit) << 60L )| (long)k0 << 40L) | ((long)k1 << 20L) | (long)k2;
	}

	public GeoTuple4D_F32<?> decode(long encoded,GeoTuple4D_F32<?> p) {
		int v = decode(encoded,tmpkey);
		return get(tmpkey,p,v);
	}

	public void insertRandom(int count) {
		Vector3D_F32 s = new Vector3D_F32(0,0,0);
		Vector3D_F32 e = new Vector3D_F32(0,0,0);
		for(int i=0; i < count; i++) {
			e.setTo((float)Math.random()*20-10, (float)Math.random()*20-10, -(float)Math.random()*4);
			update(s, e);
		}
		map.prune();
	}

	private GeoTuple4D_F32<?> get(OcTreeKeyReadOnly k,GeoTuple4D_F32<?> p, int value_4bit) {
		if(p == null)
			p = new Point4D_F32();
		map.keyToCoordinate(k, tmp);
		p.setTo(tmp.getX32(),tmp.getY32(),-tmp.getZ32(),value_4bit);
		return p;	
	}

	private void deleteNode(OcTreeKeyReadOnly key) {
		if(map.deleteNode(key)) 
			deletedNodeListenCoded.add(encode(key.getKey(0), key.getKey(1), key.getKey(2),DELETED));
	}



}
