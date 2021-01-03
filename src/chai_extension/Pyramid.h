// Pyramid.h
#ifndef CPYRAMID_H
#define CPYRAMID_H

#include <vector>
#include <Eigen/Core>
#include "chai3d.h"

namespace chai3d {

// local coordinate system:
// - origin: center of base
// - x: aligned with one vertex of the base polygon
// - z: from origin to apex

class cPyramid: public chai3d::cGenericObject {
public:
	// ctor
	/**
     * @brief Creates a visual Capsule object in the Chai graphics world.
     * @param num_sides_base Number of sides on the base polygon.
     * @param length_base_side Length of any side of the base polygon.
     * @param height Height of the pyramid.
	 * @param use_base_center_vertex Whether a vertex should be added to the center of
	 *  the base. If false, the mesh is not symmetric
     */
	cPyramid(uint num_sides_base,
		double length_base_side,
		double height,
		bool use_base_center_vertex = true
	);

	// dtor
	virtual ~cPyramid();

// protected member functions
protected:
	// render: from parent class
	virtual void render(chai3d::cRenderOptions& a_options);

// private internal functions:
	void generateLocalVertexList();

// public data members. TODO: getters, setters
public:
	/* ---- Graphic info ----*/
	uint _num_sides_base;

	double _length_base_side;

	double _height;

	bool _f_use_base_center_vertex;

	// included angle of each side of the base
	double _included_angle;

	double _side_angle;

	double _circum_radius;

	double _incircle_radius;

	Eigen::Vector3d _normal0;

	Eigen::Vector3d _apex;

	std::vector<Eigen::Vector3d> _base_vertices;
};

// function to create cMultiMesh for capsule shape
void cCreatePyramid(cMesh* a_mesh, 
    uint num_sides_base,
	double length_base_side,
	double height,
	bool use_base_center_vertex = true
);

} // namespace chai3d





#endif // CPYRAMID_H