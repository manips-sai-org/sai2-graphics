// Capsule.cpp

#include "Capsule.h"
#include <math.h>
#include <stdexcept>

// #include <iostream>
using namespace std;

using namespace Eigen;

namespace chai3d {

// get radius of the capsule at location along the capsule
double cCapsule::radius(double s) const {
	assert(s > (0-_radius-1e-10) && s < (_length+_radius+1e-10));
	if (s < 0.0) {
		double ret_rad_sqr = _radius*_radius - s*s;
		if (ret_rad_sqr < 1e-10) {
			return 0.0;
		} else {
			return sqrt(ret_rad_sqr);
		}
	}
	if (s > _length) {
		double ret_rad_sqr = _radius*_radius - (s-_length)*(s-_length);
		if (ret_rad_sqr < 1e-10) {
			return 0.0;
		} else {
			return sqrt(ret_rad_sqr);
		}
	}
	return _radius;
}

// get the local position of a point given the coordinates in polar form
void cCapsule::cartesianPoint(Eigen::Vector3d& ret_vector, const PointPolar& point) const {
	// check for correct ranges
	assert(point.s > (0-_radius-1e-10) && point.s < (_length+_radius+1e-10));
	assert(point.t > (0 - 1e-10) && point.t < (_radius+1e-10));
	assert(point.eta > (0 - 1e-10) && point.eta < (2.0*M_PI+1e-10));

	ret_vector[0] = point.s;
	ret_vector[1] = point.t * cos(point.eta);
	ret_vector[2] = point.t * sin(point.eta);
}

// get the surface normal of a point given the coordinates in polar form
void cCapsule::normal(Eigen::Vector3d& ret_vector, const PointPolar& point) const {
	// check for correct ranges
	assert(point.s > (0-_radius-1e-10) && point.s < (_length+_radius+1e-10));
	assert(point.t > (0 - 1e-10) && point.t < (_radius+1e-10));
	assert(point.eta > (0 - 1e-10) && point.eta < (2.0*M_PI+1e-10));

	Vector3d cpoint;
	if (point.s < 0.0) {
		cartesianPoint(cpoint, point);
		ret_vector << cpoint/cpoint.norm();
	} else if (point.s > _length) {
		cartesianPoint(cpoint, point);
		Vector3d rel_point = cpoint - Vector3d(_length, 0, 0);
		ret_vector << rel_point/rel_point.norm();
	} else {
		ret_vector << 0.0, cos(point.eta), sin(point.eta);
	}
}

// ctor
cCapsule::cCapsule(double radius,
		double length,
		double mesh_scale,
		uint max_longitudinal_slices,
		uint max_circumference_slices)
: _radius(radius), _length(length), _mesh_scale(mesh_scale),
_num_longitudinal_slices(max_longitudinal_slices), _num_circumferential_slices(max_circumference_slices)
{
	// default material
    m_material = cMaterial::create();
    m_material->setShininess(10);
    m_material->m_ambient.set ((float)0.3, (float)0.3, (float)0.3);
    m_material->m_diffuse.set ((float)0.1, (float)0.7, (float)0.8);
    m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
}

// dtor
cCapsule::~cCapsule() {
	// nothing to do
}

// render
void cCapsule::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL
    static Vector3d point1, point2, point3, point4;
    static PointPolar spoint1, spoint2, spoint3, spoint4;
    static Vector3d normal1, normal2, normal3, normal4;
    //^ NOTE: point 1 and point 2 lie on the current plane
    // point 3 and 4 lie on the next plane
    // point 1 and 3 lie on the current angle
    // point 2 and 4 lie on the next angle
    // NOTE: we are using per vertex normal
    

    /////////////////////////////////////////////////////////////////////////
    // ENABLE SHADER
    /////////////////////////////////////////////////////////////////////////
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // enable shader
        m_shaderProgram->use(this, a_options);
    }


    /////////////////////////////////////////////////////////////////////////
    // Render parts that use material properties
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
    {
        // render material properties
        if (m_useMaterialProperty)
        {
            m_material->render(a_options);
        }

        if (!m_displayList.render(m_useDisplayList))
        {
            // create display list if requested
            m_displayList.begin(m_useDisplayList);

            // compute spacing parameters
            double _ds_longitudinal = _length/(_num_longitudinal_slices-1);
            double _ds_longitudinal_cap = _ds_longitudinal/3.0;
			double _ds_angle = 2.0*M_PI/_num_circumferential_slices;

            uint nv_endcap_offset = max((uint)10, uint(_radius/_ds_longitudinal_cap) + (uint)1);
            _ds_longitudinal_cap = _radius/(nv_endcap_offset - 1);
            uint nv_long_wendcaps = _num_longitudinal_slices + nv_endcap_offset*2;

            double s_i, s_ip1, t_i, t_ip1;
            // cout << "_ds_longitudinal " << _ds_longitudinal << " _ds_longitudinal_cap: " << _ds_longitudinal_cap << endl;
            // cout << "_num_longitudinal_slices " << _num_longitudinal_slices << " nv_endcap_offset: " << nv_endcap_offset << " nv_long_wendcaps: " << nv_long_wendcaps << endl;
            for (uint i = 0; i < (nv_long_wendcaps-1); i++) {
                // i = index of current cross section plane
                // j = index of current vertex along the plane
                if (i < nv_endcap_offset) {
                    s_i = ((double)i)*_ds_longitudinal_cap - _radius;
                } else if (i > (nv_endcap_offset + _num_longitudinal_slices - 1)) {
                    s_i = ((double)(i - nv_endcap_offset - _num_longitudinal_slices))*_ds_longitudinal_cap + _length;
                } else {
                    s_i = ((double)(i - nv_endcap_offset))*_ds_longitudinal;
                }
                if (i+1 < nv_endcap_offset) {
                    s_ip1 = ((double)i+1)*_ds_longitudinal_cap - _radius;
                } else if (i+1 > (nv_endcap_offset + _num_longitudinal_slices - 1)) {
                    s_ip1 = ((double)(i+1 - nv_endcap_offset - _num_longitudinal_slices))*_ds_longitudinal_cap + _length;
                } else {
                    s_ip1 = ((double)(i+1 - nv_endcap_offset))*_ds_longitudinal;
                }
                s_ip1 = min(s_ip1, _length + _radius);

                // cout << "i: " << i << " s_i: " << s_i << " s_ip1 " << s_ip1 << endl;
                t_i = radius(s_i);
                t_ip1 = radius(s_ip1);

            	for (uint j = 0; j < (_num_circumferential_slices); j++) {
            		// get spatial locations of the four corners in the local
            		// frame. t != 0, eta = 0 => on the y-axis
                    spoint1 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ ((double)j)*_ds_angle);
            		cartesianPoint(point1, spoint1);
            		normal(normal1, spoint1);
                    spoint3 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ ((double)j)*_ds_angle);
					cartesianPoint(point3, spoint3);
					normal(normal3, spoint3);
            		if (j == (_num_circumferential_slices - 1)) {
            			// wrap around. so point 2 and 4 are on zero angle again
                        spoint2 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ 0.0);
						cartesianPoint(point2, spoint2);
						normal(normal2, spoint2);
                        spoint4 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ 0.0);
						cartesianPoint(point4, spoint4);
						normal(normal4, spoint4);
            		} else {
                        spoint2 = PointPolar(/* s */s_i, /* t */ t_i, /* eta */ ((double)j+1)*_ds_angle);
						cartesianPoint(point2, spoint2);
						normal(normal2, spoint2);
                        spoint4 = PointPolar(/* s */s_ip1, /* t */ t_ip1, /* eta */ ((double)j+1)*_ds_angle);
						cartesianPoint(point4, spoint4);
						normal(normal4, spoint4);
            		}

            		// add triangle 1
            		// - add vertices and per-vertex normals
					glBegin(GL_POLYGON);
						glNormal3d(normal1[0], normal1[1], normal1[2]);
						glVertex3d(point1[0], point1[1], point1[2]);
						glNormal3d(normal2[0], normal2[1], normal2[2]);
						glVertex3d(point2[0], point2[1], point2[2]);
						glNormal3d(normal3[0], normal3[1], normal3[2]);
						glVertex3d(point3[0], point3[1], point3[2]);
					glEnd();

            		// add triangle 2
            		// - add vertices and per-vertex normals
            		glBegin(GL_POLYGON);
						glNormal3d(normal2[0], normal2[1], normal2[2]);
						glVertex3d(point2[0], point2[1], point2[2]);
						glNormal3d(normal4[0], normal4[1], normal4[2]);
						glVertex3d(point4[0], point4[1], point4[2]);
						glNormal3d(normal3[0], normal3[1], normal3[2]);
						glVertex3d(point3[0], point3[1], point3[2]);
					glEnd();
            	}
            }

            // finalize display list
            m_displayList.end(true);
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // DISABLE SHADER
    /////////////////////////////////////////////////////////////////////////
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // disable shader
        m_shaderProgram->disable();
    }

#endif
}

}
