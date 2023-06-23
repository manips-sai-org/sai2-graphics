// Pyramid.cpp

#include "Pyramid.h"

#include <math.h>

#include <stdexcept>

// #include <iostream>
using namespace std;

using namespace Eigen;

namespace chai3d {

// ctor
cPyramid::cPyramid(uint num_sides_base, double length_base_side, double height,
				   bool use_base_center_vertex)
	: _num_sides_base(num_sides_base),
	  _length_base_side(length_base_side),
	  _height(height),
	  _f_use_base_center_vertex(use_base_center_vertex) {
	// default material
	m_material = cMaterial::create();
	m_material->setShininess(10);
	m_material->m_ambient.set((float)0.3, (float)0.3, (float)0.3);
	m_material->m_diffuse.set((float)0.1, (float)0.7, (float)0.8);
	m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);

	// create base vertex list
	_included_angle = 2 * M_PI / _num_sides_base;
	_circum_radius = _length_base_side / 2 / sin(_included_angle / 2);
	_incircle_radius = _length_base_side / 2 / tan(_included_angle / 2);
	_side_angle = atan2(_incircle_radius, _height);
	_normal0 << cos(_side_angle), 0, sin(_side_angle);
	_normal0 = AngleAxisd(_included_angle / 2, Vector3d::UnitZ()) * _normal0;

	_apex << 0, 0, _height;

	generateLocalVertexList();
}

// dtor
cPyramid::~cPyramid() {
	// nothing to do
}

void cPyramid::generateLocalVertexList() {
	Vector3d vertex0(_circum_radius, 0, 0);
	for (uint i = 0; i < _num_sides_base; i++) {
		_base_vertices.push_back(
			AngleAxisd(_included_angle * i, Vector3d::UnitZ()) * vertex0);
	}
}

// render
void cPyramid::render(cRenderOptions& a_options) {
#ifdef C_USE_OPENGL

	/////////////////////////////////////////////////////////////////////////
	// ENABLE SHADER
	/////////////////////////////////////////////////////////////////////////
	if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map)) {
		// enable shader
		m_shaderProgram->use(this, a_options);
	}

	/////////////////////////////////////////////////////////////////////////
	// Render parts that use material properties
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency)) {
		// render material properties
		if (m_useMaterialProperty) {
			m_material->render(a_options);
		}

		if (!m_displayList.render(m_useDisplayList)) {
			// create display list if requested
			m_displayList.begin(m_useDisplayList);

			// base
			if (_f_use_base_center_vertex) {
				Vector3d normal(0, 0, -1);
				for (uint i = 0; i < _num_sides_base - 1; i++) {
					glBegin(GL_POLYGON);
					glNormal3d(normal[0], normal[1], normal[2]);
					glVertex3d(0, 0, 0);
					glVertex3d(_base_vertices[i][0], _base_vertices[i][1],
							   _base_vertices[i][2]);
					glVertex3d(_base_vertices[i + 1][0],
							   _base_vertices[i + 1][1],
							   _base_vertices[i + 1][2]);
					glEnd();
				}
				// add last triangle
				glBegin(GL_POLYGON);
				glNormal3d(normal[0], normal[1], normal[2]);
				glVertex3d(0, 0, 0);
				glVertex3d(_base_vertices[_num_sides_base - 1][0],
						   _base_vertices[_num_sides_base - 1][1],
						   _base_vertices[_num_sides_base - 1][2]);
				glVertex3d(_base_vertices[0][0], _base_vertices[0][1],
						   _base_vertices[0][2]);
				glEnd();
			} else {
				Vector3d normal(0, 0, -1);
				Vector3d point0 = _base_vertices[0];
				for (uint i = 1; i < _num_sides_base - 1; i++) {
					glBegin(GL_POLYGON);
					glNormal3d(normal[0], normal[1], normal[2]);
					glVertex3d(point0[0], point0[1], point0[2]);
					glVertex3d(_base_vertices[i][0], _base_vertices[i][1],
							   _base_vertices[i][2]);
					glVertex3d(_base_vertices[i + 1][0],
							   _base_vertices[i + 1][1],
							   _base_vertices[i + 1][2]);
					glEnd();
				}
			}

			// sides
			for (uint i = 0; i < _num_sides_base - 1; i++) {
				Vector3d normal =
					AngleAxisd(_included_angle * i, Vector3d::UnitZ()) *
					_normal0;
				glBegin(GL_POLYGON);
				glNormal3d(normal[0], normal[1], normal[2]);
				glVertex3d(_apex[0], _apex[1], _apex[2]);
				glVertex3d(_base_vertices[i][0], _base_vertices[i][1],
						   _base_vertices[i][2]);
				glVertex3d(_base_vertices[i + 1][0], _base_vertices[i + 1][1],
						   _base_vertices[i + 1][2]);
				glEnd();
			}
			// last triangle
			{
				Vector3d normal =
					AngleAxisd(_included_angle * (_num_sides_base - 1),
							   Vector3d::UnitZ()) *
					_normal0;
				glBegin(GL_POLYGON);
				glNormal3d(normal[0], normal[1], normal[2]);
				glVertex3d(_apex[0], _apex[1], _apex[2]);
				glVertex3d(_base_vertices[_num_sides_base - 1][0],
						   _base_vertices[_num_sides_base - 1][1],
						   _base_vertices[_num_sides_base - 1][2]);
				glVertex3d(_base_vertices[0][0], _base_vertices[0][1],
						   _base_vertices[0][2]);
				glEnd();
			}

			// finalize display list
			m_displayList.end(true);
		}
	}

	/////////////////////////////////////////////////////////////////////////
	// DISABLE SHADER
	/////////////////////////////////////////////////////////////////////////
	if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map)) {
		// disable shader
		m_shaderProgram->disable();
	}

#endif
}

}  // namespace chai3d
