package com.comino.mavmap.map.map3D.impl.octomap.test;

import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;

public class OctoMapTest {

	MAVOccupancyOcTree map;

	public OctoMapTest() {

		map = new MAVOccupancyOcTree(0.2);

	}
	

	public void fillTwo(boolean set) {
		map.updateNode((float)(5/5.0f), (float)(5/5.0f), (float)(5/5.0f),true);
		map.updateNode((float)(10/5.0f), (float)(5/5.0f), (float)(5/5.0f),true);
		System.out.println("Nodes for One at 1,1,1 and 2,1,1: "+map.getNumberOfNodes());
	}
	
	public void removeOne() {
		OcTreeKey k = OcTreeKeyConversionTools.coordinateToKey((float)(5/5.0f), (float)(5/5.0f), (float)(5/5.0f), map.getResolution(),map.getTreeDepth());
		map.deleteNode(k);
		if(map.getNumberOfNodes()==1) {
			map.clear();
		}
		System.out.println("Nodes for One at 1,1,1: "+map.getNumberOfNodes());
	}

	public void fill10x10x3() {

		for(int x=0; x<50;x++) {
			for(int y=0; y<50;y++) {
				for(int z=0; z<15;z++) {
					map.updateNode((float)(x/5.0f), (float)(y/5.0f), (float)(z/5.0f),true);
				}
			}
		}

		System.out.println("Nodes for10x10x3: "+map.getNumberOfNodes());
	}

	public void remove1x1x1() {

		for(int x=0; x<5;x++) {
			for(int y=0; y<5;y++) {
				for(int z=0; z<5;z++) {
					map.updateNode((float)(x/5.0f), (float)(y/5.0f), (float)(z/5.0f),1e-7f);
				}
			}
		}
		System.out.println("Nodes after 1x1x1 remove: "+map.getNumberOfNodes());
	}
	
	public void delete5x5x3() {

		for(int x=0; x<25;x++) {
			for(int y=0; y<25;y++) {
				for(int z=0; z<15;z++) {
					OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey((float)(x/5.0f), (float)(y/5.0f), (float)(z/5.0f), 0.2, 16);
					map.deleteNode(key);
				}
			}
		}
		System.out.println("Nodes after 5x5x1 delete: "+map.getNumberOfNodes());
	}
	
	public void getNoNodes5x5x3() {
		
		OcTreeBoundingBoxWithCenterAndYaw b = new OcTreeBoundingBoxWithCenterAndYaw(new Point3D(0,0,0),new Point3D(5,5,3),0.2,16);
		int size = OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), b).toList().size();
		System.out.println("Nodes in boundingbox 5x5x3: "+size);
		
	}
	
	public void prune() {
		map.prune();
		System.out.println("Nodes after pruning: "+map.getNumberOfNodes());
	}



	public static void main(String[] args) {
		OctoMapTest test = new OctoMapTest();

		test.fillTwo(true);
		test.removeOne();
		
		test.fillTwo(true);
		test.removeOne();
		

		test.fillTwo(true);
		test.removeOne();
	

	}

}
