// Capsule mesh

#include "Capsule.h"

using namespace std;

using namespace Eigen;

namespace chai3d {

void cCreateCapsule(cMesh* a_mesh, 
    double a_radius,
    double a_length,
    uint a_num_longitudinal_slices,
    uint a_num_circumferential_slices
) {
	cCapsule obj(a_radius, a_length, 0.1, a_num_longitudinal_slices, a_num_circumferential_slices);
	Vector3d point1, point2, point3, point4;
    PointPolar spoint1, spoint2, spoint3, spoint4;
    Vector3d normal1, normal2, normal3, normal4;

    // compute spacing parameters
    double _ds_longitudinal = a_length/(a_num_longitudinal_slices-1);
    double _ds_longitudinal_cap = _ds_longitudinal/3.0;
	double _ds_angle = 2.0*M_PI/a_num_circumferential_slices;

    uint nv_endcap_offset = max((uint)10, uint(a_radius/_ds_longitudinal_cap) + (uint)1);
    _ds_longitudinal_cap = a_radius/(nv_endcap_offset - 1);
    uint nv_long_wendcaps = a_num_longitudinal_slices + nv_endcap_offset*2;

    double s_i, s_ip1, t_i, t_ip1;
    // cout << "_ds_longitudinal " << _ds_longitudinal << " _ds_longitudinal_cap: " << _ds_longitudinal_cap << endl;
    // cout << "a_num_longitudinal_slices " << a_num_longitudinal_slices << " nv_endcap_offset: " << nv_endcap_offset << " nv_long_wendcaps: " << nv_long_wendcaps << endl;
    for (uint i = 0; i < (nv_long_wendcaps-1); i++) {
        // i = index of current cross section plane
        // j = index of current vertex along the plane
        if (i < nv_endcap_offset) {
            s_i = ((double)i)*_ds_longitudinal_cap - a_radius;
        } else if (i > (nv_endcap_offset + a_num_longitudinal_slices - 1)) {
            s_i = ((double)(i - nv_endcap_offset - a_num_longitudinal_slices))*_ds_longitudinal_cap + a_length;
        } else {
            s_i = ((double)(i - nv_endcap_offset))*_ds_longitudinal;
        }
        if (i+1 < nv_endcap_offset) {
            s_ip1 = ((double)i+1)*_ds_longitudinal_cap - a_radius;
        } else if (i+1 > (nv_endcap_offset + a_num_longitudinal_slices - 1)) {
            s_ip1 = ((double)(i+1 - nv_endcap_offset - a_num_longitudinal_slices))*_ds_longitudinal_cap + a_length;
        } else {
            s_ip1 = ((double)(i+1 - nv_endcap_offset))*_ds_longitudinal;
        }
        s_ip1 = min(s_ip1, a_length + a_radius);

        // cout << "i: " << i << " s_i: " << s_i << " s_ip1 " << s_ip1 << endl;
        t_i = obj.radius(s_i);
        t_ip1 = obj.radius(s_ip1);

    	for (uint j = 0; j < (a_num_circumferential_slices); j++) {
    		// get spatial locations of the four corners in the local
    		// frame. t != 0, eta = 0 => on the y-axis
            spoint1 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ ((double)j)*_ds_angle);
    		obj.cartesianPoint(point1, spoint1);
    		obj.normal(normal1, spoint1);
            spoint3 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ ((double)j)*_ds_angle);
			obj.cartesianPoint(point3, spoint3);
			obj.normal(normal3, spoint3);
    		if (j == (a_num_circumferential_slices - 1)) {
    			// wrap around. so point 2 and 4 are on zero angle again
                spoint2 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ 0.0);
				obj.cartesianPoint(point2, spoint2);
				obj.normal(normal2, spoint2);
                spoint4 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ 0.0);
				obj.cartesianPoint(point4, spoint4);
				obj.normal(normal4, spoint4);
    		} else {
                spoint2 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ ((double)j+1)*_ds_angle);
				obj.cartesianPoint(point2, spoint2);
				obj.normal(normal2, spoint2);
                spoint4 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ ((double)j+1)*_ds_angle);
				obj.cartesianPoint(point4, spoint4);
				obj.normal(normal4, spoint4);
    		}

			// create new vertices
		    int vertexIndex0 = a_mesh->newVertex(cVector3d(point1), cVector3d(normal1));
		    int vertexIndex1 = a_mesh->newVertex(cVector3d(point2), cVector3d(normal2));
		    int vertexIndex2 = a_mesh->newVertex(cVector3d(point3), cVector3d(normal3));
		    int vertexIndex3 = a_mesh->newVertex(cVector3d(point4), cVector3d(normal4));

		    // create triangles
		    a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
		    a_mesh->newTriangle(vertexIndex1, vertexIndex3, vertexIndex2);
    	}
    }
}

}
