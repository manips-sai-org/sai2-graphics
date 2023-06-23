// Capsule.h: Chai visual for a capsule element

#ifndef CCAPSULE_H
#define CCAPSULE_H

#include <Eigen/Core>

#include "chai3d.h"

namespace chai3d {

// co-ordinates of 3d space within the capsule's surface using polar
// representation
struct PointPolar {
	double s;	 // length along the capsule
	double t;	 // radial distance in the planar frame at s
	double eta;	 // angle subtended with X axis in planar frame at s
	// ctor
	PointPolar(double ss = 0.0, double st = 0.0, double seta = 0.0)
		: s(ss), t(st), eta(seta) { /* Nothing to do */
	}
};

// NOTE:
// Pose of frame with respect to parent: The origin is located at one end of the
// capsule's defining line segment. The X axis points towards the length of the
// capsule. Y and Z axes are default, orthogonal to X.

class cCapsule : public chai3d::cGenericObject {
	// public member functions
public:
	// ctor
	/**
	 * @brief Creates a visual Capsule object in the Chai graphics world.
	 * @param radius Radius of the capsule.
	 * @param length Length of the line segment defining the capsule. This is
	 * not the extent of the capsule which equals length + 2* radius.
	 * @param mesh_scale User constraint on the ratio of minimum mesh edge
	 * length to the length or diameter of the capsule, whichever is smaller.
	 * Controls how fine the mesh is.
	 * @param max_longitudinal_slices Maximum number of mesh slices along length
	 * of capsule
	 * @param max_circumference_slices Maximum number of mesh slices along
	 * single cross section arc.
	 */
	cCapsule(double radius, double length, double mesh_scale = 0.1,
			 uint max_longitudinal_slices = 20,
			 uint max_circumference_slices = 16);

	// dtor
	virtual ~cCapsule();

	// protected member functions
protected:
	// render: from parent class
	virtual void render(chai3d::cRenderOptions& a_options);

	// internal functions but public
public:
	// radius of cross section at a point along line segment.
	// @param s Length along line segment from 0 to length.
	double radius(double s) const;

	// get the local position of a point given the coordinates in polar form
	void cartesianPoint(Eigen::Vector3d& ret_vector,
						const PointPolar& point) const;

	// get the projected surface normal of a point given the coordinates in
	// polar form
	void normal(Eigen::Vector3d& ret_vector, const PointPolar& point) const;

	// public data members. TODO: getters, setters
public:
	/* ---- Graphic info ----*/
	// Radius of the capsule.
	double _radius;

	// Length of the line segment defining the capsule.
	double _length;

	// User constraint on the ratio of minimum mesh edge length to the
	//  length or diameter of the capsule, whichever is smaller.
	double _mesh_scale;

	// number of vertices in a single plane cross section
	uint _num_circumferential_slices;

	// number of plane cross sections along the length of the capsule
	uint _num_longitudinal_slices;
};

// function to create cMultiMesh for capsule shape
void cCreateCapsule(cMesh* a_mesh, double a_radius, double a_length,
					uint a_num_longitudinal_slices = 20,
					uint a_num_circumferential_slices = 16,
					cColorf color1 = cColorf(0.7, 0.48, 0.18),
					cColorf color2 = cColorf(0.2, 0.05, 0.0));

}  // namespace chai3d

#endif	// CCAPSULE_H
