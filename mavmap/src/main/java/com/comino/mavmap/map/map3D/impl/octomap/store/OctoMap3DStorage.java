package com.comino.mavmap.map.map3D.impl.octomap.store;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.comino.mavcom.config.MSPConfig;
import com.comino.mavmap.map.map3D.Map3DSpacialInfo;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTree;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOccupancyOcTreeNode;
import com.comino.mavmap.map.map3D.impl.octomap.MAVOctoMap3D;
import com.comino.mavutils.MSPMathUtils;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point3D_I32;
import georegression.struct.point.Point4D_F32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.OccupancyOcTreeNode;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.jOctoMap.tools.OcTreeNodeTools;

public class OctoMap3DStorage {

	private final static String EXT           = ".m3D";
	private final static String EXT_OCTOMAP   = ".bt";

	private  MAVOctoMap3D 			   map;
	private  String                    base_path;

	private String id;
	private float resolution;
	private long size;
	private long node_counter;
	private float lat;
	private float lon;
	private String filename;
	private Gson gson;

	private static byte[] work = new byte[8];



	public OctoMap3DStorage(MAVOctoMap3D map, double lat, double lon) {

		this.map = map;

		this.lat = (float)Math.floor(lat * 1000000d) / 1000000f;
		this.lon = (float)Math.floor(lon * 1000000d) / 1000000f;

		try {
			this.base_path = MSPConfig.getInstance().getBasePath()+"/";
		} catch(Exception e) {
			this.base_path = System.getProperty("user.home")+"/";
		}

		this.filename = generateFileName()+EXT_OCTOMAP;
		this.gson = new GsonBuilder().serializeSpecialFloatingPointValues().create();

	}


	public MAVOctoMap3D importOctomap(String filename) throws IOException {

		File f = new File(base_path+filename);
		if(!f.exists()) {
			System.err.println("File "+filename+" does not exist in "+base_path);
			return map;
		}

		InputStream inputStream = new FileInputStream(f);

		if(!readHeader(inputStream)) {
			System.err.println("File "+filename+" does not have the correct format");
			return map ;
		}

		System.out.println("File "+filename+": id="+id+" resolution="+resolution+" size="+size);

		map.clear();
		map.resetChangeDetection();
		node_counter = 0;
		map.getTree().updateNode(OcTreeKeyTools.getRootKey(map.getTree().getTreeDepth()),false);

		readOTDataRecursive(map.getTree(),null, inputStream);

		inputStream.close();

		System.out.println("\n"+map.getTree().numberOfChangesDetected()+" nodes read from "+filename);
		return map;

	}

	public void write() {

		List<Long> data = map.getTreeEncoded();

		File f = new File(base_path+filename);
		System.out.println("Map stored to "+f.getPath());
		if(f.exists()) f.delete();
		try {
			f.createNewFile();
			FileOutputStream fs = new FileOutputStream(f);
			fs.write(gson.toJson(data).getBytes());
			fs.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		data.clear();
	}

	public boolean locateAndRead() {
		return locateAndRead(this.lat, this.lon);
	}


	public boolean locateAndRead(float lat, float lon) {
		float[] origin; String found; float distance_origin, distance = Float.MAX_VALUE;
		float[] req_translation = new float[2];

		MSPMathUtils.map_projection_init(this.lat, this.lon);


		found = null;
		for( String f : getMapFileNames()) {

			if(f.contains("test") || f.contains("7fc000007fc00000")) {
				found = f;
				break;
			}

			origin = getOriginFromFileName(f);
			distance_origin = MSPMathUtils.map_projection_distance(lat, lon, origin[0], origin[1], req_translation);
			if(distance_origin<distance ) {
				found = f; distance = distance_origin;
			}
		}

		if(found==null)
			return false;

		return read(found);
	}


	private boolean read(String fn) {

		Point4D_F32 mappo = new Point4D_F32();

		map.clear();
		map.resetChangeDetection();
		node_counter = 0;
		map.getTree().updateNode(OcTreeKeyTools.getRootKey(map.getTree().getTreeDepth()),false);

		File f = new File(base_path+fn);
		if(f.exists()) {
			try {
				FileInputStream fs = new FileInputStream(f);
				Long[] data = gson.fromJson(new BufferedReader(new InputStreamReader(fs)), Long[].class);
				System.out.println("Map '"+f.getAbsolutePath()+"' found in store with "+data.length+ " entries");
				for(int i=0;i<data.length;i++) {
					map.decode(data[i], mappo);
					map.insert(mappo);
				}
				return true;
			} catch (Exception e) {
				System.err.println(fn+" reading error ");
				e.printStackTrace();
				return false;
			}
		}
		System.err.println(fn+" not found");
		return false;
	}




	public boolean readLegacyM3D(String fn) {

		Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().create();

		Map3DSpacialInfo info = new Map3DSpacialInfo(0.10f,20.0f,20.0f,5.0f);

		map.clear();
		map.resetChangeDetection();
		node_counter = 0;
		map.getTree().updateNode(OcTreeKeyTools.getRootKey(map.getTree().getTreeDepth()),false);

		Point3D_I32 mappo = new Point3D_I32(); Point3D_F64 global = new Point3D_F64();
		double prob =0;
		File f = new File(base_path+fn);
		if(f.exists()) {
			try {
				FileInputStream fs = new FileInputStream(f);
				Long[] data = gson.fromJson(new BufferedReader(new InputStreamReader(fs)), Long[].class);
				System.out.println("Map '"+f.getAbsolutePath()+"' found in store with "+data.length+ " entries");
				for(int i=0;i<data.length;i++) {
					prob = info.decodeMapPoint(data[i], mappo);
					info.mapToGlobal(mappo, global);
					if(prob> 0.5)
						map.insert((float)global.x,(float)global.y,(float)global.z);
				}
				return true;
			} catch (Exception e) {
				System.err.println(fn+" reading error ");
				e.printStackTrace();
				return false;
			}
		}
		System.err.println(fn+" not found");
		return false;
	}

	private String generateFileName() {
		return Integer.toHexString(Float.floatToIntBits(lat)) + Integer.toHexString(Float.floatToIntBits(lon));
	}

	private float[] getOriginFromFileName(String filename) {
		float[] r = new float[2];
		r[0] =  Float.intBitsToFloat(Integer.decode("0x"+filename.substring(0, 8)));
		r[1] =  Float.intBitsToFloat(Integer.decode("0x"+filename.substring(8, 16)));
		return r;
	}

	private List<String> getMapFileNames() {
		ArrayList<String> result = new ArrayList<String>();
		File folder = new File(base_path);
		for(File f : folder.listFiles()) {
			if(f.isFile() && f.getName().contains(EXT_OCTOMAP))
				result.add(f.getName());
		}
		return result;
	}

	private void readOTDataRecursive(MAVOccupancyOcTree tmp, MAVOccupancyOcTreeNode node, InputStream in) {

		try {

			if(node == null)
				node = tmp.getRoot();

			if(in.available()<=0 )
				return;

			node_counter++;
			if(node_counter % 100000 == 0)
				System.out.print(".");

			float f = Float.intBitsToFloat(readInt(in));
			node.setLogOdds(f);


			// Read Childrenbits
			byte children = (byte)in.read();

			if(!node.hasArrayForChildren())
				node.allocateChildren();

			int tree_depth = tmp.getTreeDepth();
			int childDepth = node.getDepth()+1;

			for(int i = 0; i < 8; i++) {
				if( ((children >> i) & 0x1 )== 0x1 ) {
					MAVOccupancyOcTreeNode new_node = new MAVOccupancyOcTreeNode();
					OcTreeKey childKey = OcTreeKeyTools.computeChildKey(i, node, childDepth,tree_depth);



					//					tmp.keyToCoordinate(childKey, point);
					//					System.out.println(point+":"+childDepth+":"+OcTreeKeyTools.computeCenterOffsetKey(tree_depth));

					tmp.getChangedKeys().put(childKey, true);
					new_node.setProperties(childKey, childDepth, resolution, tree_depth);
					node.setChild(i, new_node);
					readOTDataRecursive(tmp,new_node,in);
				}
			}
		} catch(Exception e) {
			System.err.println(e.getMessage());
			System.exit(1);;
		}
	}

	private boolean readHeader(InputStream in) throws IOException {
		while(in.available()>0) {
			String s = nextToken(in);
			if(s.compareToIgnoreCase("data")==0) {
				return true;
			} 
			if( s.compareToIgnoreCase("id")==0) {
				this.id = nextToken(in);
			}  
			if( s.compareToIgnoreCase("res")==0) {
				this.resolution = Float.parseFloat(nextToken(in));
			}  
			if( s.compareToIgnoreCase("size")==0) {
				this.size = Long.parseLong(nextToken(in));
			}  
		}
		return false;
	}

	private void readEOL(InputStream in) throws IOException {
		char c;
		while((c=(char)in.read())!='\n') System.out.print(c);
		System.out.println();
	}

	private String nextToken(InputStream in) throws IOException {
		String s=""; char c;
		while((c = (char)in.read())!=-1 && c!= ' ' && c!='\n') s += c;
		return s;
	}

	private int readInt(InputStream in) throws IOException {
		in.read(work, 0, 4);
		return (int)(work[3]) << 24 | (work[2] & 0xff) << 16 | (work[1] & 0xff) << 8 | (work[0] & 0xff);
	}

	//	private int readInt(InputStream in) throws IOException {
	//		in.read(work, 0, 4);
	//		return (int)(work[0]) << 24 | (work[1] & 0xff) << 16 | (work[2] & 0xff) << 8 | (work[3] & 0xff);
	//	}

	//	private long readLong(InputStream in) throws IOException {
	//		in.read(work, 0, 8);
	//		return (long) (work[7]) << 56 |
	//				/* long cast needed or shift done modulo 32 */
	//				(long) (work[6] & 0xff) << 48 | (long) (work[5] & 0xff) << 40 | (long) (work[4] & 0xff) << 32
	//				| (long) (work[3] & 0xff) << 24 | (long) (work[2] & 0xff) << 16 | (long) (work[1] & 0xff) << 8
	//				| (long) (work[0] & 0xff);
	//	}


	public static void main(String[] args) throws Exception {
		MAVOctoMap3D map = new MAVOctoMap3D();
		OctoMap3DStorage store = new OctoMap3DStorage(map,0,0);
		store.importOctomap("new_college.ot");
		System.out.println("Size is: "+map.getNumberOfNodes());
	}

}
