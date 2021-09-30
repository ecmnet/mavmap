/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package bubo.construct;

import georegression.struct.point.Point3D_I32;
import georegression.struct.shapes.Box3D_I32;

/**
 * Base class for constructing {@link Octree_I32}.
 *
 * @author Peter Abeles
 */
public abstract class ConstructOctree_I32 extends ConstructOctree<Octree_I32,Point3D_I32>{

	public ConstructOctree_I32() {
		super(Octree_I32.class);
	}

	/**
	 * Initializes the Octree.  The space contained by the Octree is specified by the passed in cube.
	 * {@link #reset} is automatically called by this function
	 *
	 * @param cube Space which is contained by the Octree.
	 */
	public void initialize(Box3D_I32 cube) {
		reset();
		tree.space.setTo(cube);
	}

	@Override
	public void setChildSpace(Octree_I32 parent, int index, Octree_I32 child) {
		setChildSpace(parent.space,parent.divider,index,child.space);
	}

	/**
	 * Sets the divider to the center of space
	 */
	public static void computeDivider(Box3D_I32 space, Point3D_I32 divider) {

		divider.x = (space.p0.x + space.p1.x) / 2;
		divider.y = (space.p0.y + space.p1.y) / 2;
		divider.z = (space.p0.z + space.p1.z) / 2;
	}

	public static void setChildSpace(Box3D_I32 parentSpace, Point3D_I32 parentDivider, int index,
									 Box3D_I32 childSpace) {

		childSpace.p0.setTo(parentSpace.p0);
		childSpace.p1.setTo(parentSpace.p1);

		if( index == 0 ) {
			childSpace.p1.setTo(parentDivider);
		} else if (index == 1) {
			childSpace.p0.y = parentDivider.y;
			childSpace.p1.x = parentDivider.x;
			childSpace.p1.z = parentDivider.z;
		} else if (index == 2) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p1.y = parentDivider.y;
			childSpace.p1.z = parentDivider.z;
		} else if (index == 3) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p0.y = parentDivider.y;
			childSpace.p1.z = parentDivider.z;
		} else if (index == 4) {
			childSpace.p0.z = parentDivider.z;
			childSpace.p1.x = parentDivider.x;
			childSpace.p1.y = parentDivider.y;
		} else if (index == 5) {
			childSpace.p0.y = parentDivider.y;
			childSpace.p0.z = parentDivider.z;
			childSpace.p1.x = parentDivider.x;
		} else if (index == 6) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p0.z = parentDivider.z;
			childSpace.p1.y = parentDivider.y;
		} else if (index == 7) {
			childSpace.p0.setTo(parentDivider);
		}
	}

	/**
	 * Checks to see if the provided cube has a non-zero positive volume.
	 */
	@Override
	public boolean isSpaceValid( Octree_I32 node ) {
		Box3D_I32 space = node.space;
		return space.p0.x < space.p1.x && space.p0.y < space.p1.y && space.p0.z < space.p1.z;
	}
}
