package com.comino.mavmap.map.map3D.impl.octomap.esdf.D2;

import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTreeNode;
import com.comino.mavmap.map.map3D.impl.octomap.boundingbox.MAVBoundingBox;
import com.comino.mavmap.map.map3D.impl.octomap.boundingbox.MAVSimpleBoundingBox;

import georegression.struct.GeoTuple4D_F32;
import us.ihmc.jOctoMap.iterators.OcTreeIterable.OcTreeIterator;
import us.ihmc.jOctoMap.tools.OccupancyTools;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;

import java.util.Iterator;

public class DynamicESDF2D {

	private static final int      OCCUPPIED = -1;
	private static final int      UNKNOWN   = -2;

	private final int[][]            esdf_map;

	private float                    size_x;
	private float                    size_y;

	private final int                sizex;   
	private final int                sizey;    
	private final int                sizex_2;   
	private final int                sizey_2;    

	private MAVSimpleBoundingBox     boundingBox;
	private float                    resolution;


	public DynamicESDF2D(float size) {

		this.resolution = 0.2f;
		this.boundingBox = new MAVSimpleBoundingBox(resolution,16);

		this.size_x = size;
		this.size_y = size;

		sizex = (int)(size_x / resolution) + 1;
		sizey = (int)(size_y / resolution) + 1;

		esdf_map = new int[sizex][sizey];

		initialize();

		this.sizex_2 = sizex / 2;
		this.sizey_2 = sizey / 2;

	}

	public int gezSizeX() {
		return sizex;
	}

	public int gezSizeY() {
		return sizey;
	}

	public GeoTuple4D_F32<?> getCenter() {
		return boundingBox.getCenter();
	}

	public void update(GeoTuple4D_F32<?> p,MAVOccupancyOcTree map) {
		initialize();
		boundingBox.set(p,10.0f,0.25f);	
		OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), boundingBox).forEach((node) -> {
			if(OccupancyTools.isNodeOccupied(map.getOccupancyParameters(), node)) {
				int x = getCell(node.getX(),boundingBox.getCenter().x)+sizex_2;
				int y = getCell(node.getY(),boundingBox.getCenter().y)+sizey_2; 
				if(x >= 0 && x < sizex && y >= 0 && y < sizey)
					esdf_map[x][y] = OCCUPPIED;
			}
		});

		esdf_map[sizex_2][sizey_2] = UNKNOWN;

		for(int x = -sizex_2; x < sizex_2; x++ ) {
			for(int y = -sizey_2; y < sizey_2; y++ ) {
				if(esdf_map[x+sizex_2][y+sizey_2] == OCCUPPIED)
					determineMinDistCell(x,y);
			}	
		}
	}

	public int[][] getESDF2DMap() {
		return esdf_map;
	}

	public double getDistanceAt(GeoTuple4D_F32<?> p) {
		int val = esdf_map[(int)(p.x/resolution)+sizex_2][(int)(p.y/resolution)+sizey_2];
		if(val == OCCUPPIED)
			return 0;
		if(val == UNKNOWN)
			return Float.NaN;
		return (float)(Math.sqrt(val)*resolution);
	}

	private void determineMinDistCell(int esdfx, int esdfy) {
		int distance;
		for(int x = -sizex_2; x < sizex_2; x++ ) {
			for(int y = -sizey_2; y < sizey_2; y++ ) {		
				if(esdf_map[x+sizex_2][y+sizey_2] == OCCUPPIED) {
					continue;
				}
				distance = (x - esdfx)* (x - esdfx) +  (y - esdfy)* (y - esdfy);
				if(esdf_map[x+sizex_2][y+sizey_2] == UNKNOWN) {
					esdf_map[x+sizex_2][y+sizey_2] = distance;
					continue;
				}
				if(esdf_map[x+sizex_2][y+sizey_2] > distance) {
					esdf_map[x+sizex_2][y+sizey_2] = distance;
				}
			}
		}
	}

	private int getCell(double x, double x0) {
		return (int)((x-x0)/resolution);
	}

	private void initialize() {

		for(int y=0;y < sizey;y++)
			for(int x=0;x <sizex;x++)
				esdf_map[x][y] = UNKNOWN;
	}

}
