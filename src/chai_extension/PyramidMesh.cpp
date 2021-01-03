// PyramidMesh.cpp

#include "Pyramid.h"

using namespace std;

using namespace Eigen;

namespace chai3d {

void cCreatePyramid(cMesh* a_mesh, 
    uint num_sides_base,
	double length_base_side,
	double height,
	bool use_base_center_vertex
) {
	cPyramid pyr(num_sides_base, length_base_side, height, use_base_center_vertex);
	cColorf vcolor(0.1, 0.7, 0.8);

	// base
    if(use_base_center_vertex) {
    	Vector3d normal(0,0,-1);
    	for(uint i = 0; i < num_sides_base-1; i++) {
			int vertexIndex0 = a_mesh->newVertex(cVector3d(0,0,0), cVector3d(normal), cVector3d(0,0,0), vcolor);
			int vertexIndex1 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i]), cVector3d(normal), cVector3d(0,0,0), vcolor);
			int vertexIndex2 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i+1]), cVector3d(normal), cVector3d(0,0,0), vcolor);

			// create triangle
			a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    	}
    	// add last triangle
		int vertexIndex0 = a_mesh->newVertex(cVector3d(0,0,0), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex1 = a_mesh->newVertex(cVector3d(pyr._base_vertices[num_sides_base-1]), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex2 = a_mesh->newVertex(cVector3d(pyr._base_vertices[0]), cVector3d(normal), cVector3d(0,0,0), vcolor);

		// create triangle
		a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    } else {
    	Vector3d normal(0,0,-1);
    	Vector3d point0 = pyr._base_vertices[0];
    	for(uint i = 1; i < num_sides_base-1; i++) {
			int vertexIndex0 = a_mesh->newVertex(cVector3d(point0), cVector3d(normal), cVector3d(0,0,0), vcolor);
			int vertexIndex1 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i]), cVector3d(normal), cVector3d(0,0,0), vcolor);
			int vertexIndex2 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i+1]), cVector3d(normal), cVector3d(0,0,0), vcolor);

			// create triangle
			a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    	}
    }

    // sides
    for(uint i = 0; i < num_sides_base-1; i++) {
    	Vector3d normal = AngleAxisd(pyr._included_angle*i, Vector3d::UnitZ()) * pyr._normal0;
    	int vertexIndex0 = a_mesh->newVertex(cVector3d(pyr._apex), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex1 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i]), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex2 = a_mesh->newVertex(cVector3d(pyr._base_vertices[i+1]), cVector3d(normal), cVector3d(0,0,0), vcolor);

		// create triangles
		a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    }
    // last triangle
    {
    	Vector3d normal = AngleAxisd(pyr._included_angle*(num_sides_base-1), Vector3d::UnitZ()) * pyr._normal0;
    	int vertexIndex0 = a_mesh->newVertex(cVector3d(pyr._apex), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex1 = a_mesh->newVertex(cVector3d(pyr._base_vertices[num_sides_base-1]), cVector3d(normal), cVector3d(0,0,0), vcolor);
		int vertexIndex2 = a_mesh->newVertex(cVector3d(pyr._base_vertices[0]), cVector3d(normal), cVector3d(0,0,0), vcolor);

		// create triangles
		a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    }
}

}