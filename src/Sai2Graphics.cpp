/**
 * \file Sai2Graphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "Sai2Graphics.h"
#include "parser/UrdfToSai2Graphics.h"
#include <iostream>

using namespace std;
using namespace chai3d;

namespace Sai2Graphics 
{

Sai2Graphics::Sai2Graphics(const std::string& path_to_world_file,
							bool verbose)
{
	// initialize a chai world
	_world = new chai3d::cWorld();
	Parser::UrdfToSai2GraphicsWorld(path_to_world_file, _world, verbose);
}

// dtor
Sai2Graphics::~Sai2Graphics() {
	delete _world;
	_world = NULL;	
}

static void updateGraphicsLink(cRobotLink* link, Sai2Model::Sai2Model* robot_model) {
	cVector3d local_pos;
	cMatrix3d local_rot;

	// get link name
	std::string link_name = link->m_name;

	// get chai parent name
	cGenericObject* parent = link->getParent();
	std::string parent_name = parent->m_name;

	// if parent is cRobotBase, then simply get transform relative to base from model
	if (dynamic_cast<cRobotBase*> (parent) != NULL) {
		Eigen::Affine3d T;
		robot_model->transform(T, link_name);
		local_pos = cVector3d(T.translation());
		local_rot = cMatrix3d(T.rotation());
	} else if (dynamic_cast<cRobotLink*> (parent) != NULL) {
		// if parent is cRobotLink, then calculate transform for both links, then apply inverse transform
		Eigen::Affine3d T_me, T_parent, T_rel;
		robot_model->transform(T_me, link_name);
		robot_model->transform(T_parent, parent_name);
		T_rel = T_parent.inverse() * T_me;
		local_pos = cVector3d(T_rel.translation());
		local_rot = cMatrix3d(T_rel.rotation());
	} else {
		cerr << "Parent to link " << link_name << " is neither link nor base" << endl;
		abort();
		// TODO: throw exception
	}
	link->setLocalPos(local_pos);
	link->setLocalRot(local_rot);

	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < link->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(link->getChild(i));
		if (child != NULL) {
			updateGraphicsLink(child, robot_model);
		}
	}
}

// update frame for a particular robot
void Sai2Graphics::updateGraphics(const std::string& robot_name,
									Sai2Model::Sai2Model* robot_model) {
	// get robot base object in chai world
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}
	// recursively update graphics for all children
	cRobotLink* link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (link != NULL) {
			updateGraphicsLink(link, robot_model);
		}
	}
	// update shadow maps. TODO: consider moving out from here if it is too expensive
	_world->updateShadowMaps();
}

void Sai2Graphics::updateObjectGraphics(const std::string& object_name,
                         const Eigen::Vector3d& object_pos,
                         const Eigen::Quaterniond object_ori)
{
	cGenericObject* object = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (object_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			object = _world->getChild(i);
			if (object != NULL) {
				break;
			}
		}
	}
	if (object == NULL) {
		//TODO: throw exception
		cerr << "Could not find object in chai world: " << object_name << endl;
		abort();
	}

	// update pose
	object->setLocalPos(object_pos);
	object->setLocalRot(object_ori.toRotationMatrix());


}

void Sai2Graphics::render(const std::string& camera_name,
							int window_width, 
							int window_height, 
							int display_context_id) {
	auto camera = getCamera(camera_name);
	// TODO: support link mounted cameras
	// TODO: support stereo. see cCamera::renderView
	//	to do so, we need to search through the descendent tree
	// render view from this camera
	// NOTE: we don't use the display context id right now since chai no longer
	// supports it in 3.2.0
	camera->renderView(window_width, window_height);
}

// get current camera pose
void Sai2Graphics::getCameraPose(const std::string& camera_name,
									Eigen::Vector3d& ret_position,
									Eigen::Vector3d& ret_vertical,
									Eigen::Vector3d& ret_lookat) {
	auto camera = getCamera(camera_name);
	cVector3d pos, vert, lookat;
	pos = camera->getLocalPos(); ret_position << pos.x(), pos.y(), pos.z();
	vert = camera->getUpVector(); ret_vertical << vert.x(), vert.y(), vert.z();
	lookat = camera->getLookVector(); ret_lookat << lookat.x(), lookat.y(), lookat.z();
	ret_lookat += ret_position;
}

// set camera pose
void Sai2Graphics::setCameraPose(const std::string& camera_name,
									const Eigen::Vector3d& position,
									const Eigen::Vector3d& vertical,
									const Eigen::Vector3d& lookat) {
 	auto camera = getCamera(camera_name);
	cVector3d pos(position[0], position[1], position[2]);
	cVector3d vert(vertical[0], vertical[1], vertical[2]);
	cVector3d look(lookat[0], lookat[1], lookat[2]);
	camera->set(pos, look, vert);
}

bool Sai2Graphics::getRobotLinkInCamera(const std::string& camera_name,
		                                   const std::string& robot_name,
		                                   int view_x,
		                                   int view_y,
		                                   int window_width,
										   int window_height,
		                                   std::string& ret_link_name,
		                                   Eigen::Vector3d& ret_pos) {
	cCollisionRecorder selectionRecorder;

	//use standard settings
	cCollisionSettings defaultSettings;
	defaultSettings.m_checkForNearestCollisionOnly = false;
	defaultSettings.m_checkVisibleObjects = true;
	defaultSettings.m_collisionRadius = 0;
	defaultSettings.m_returnMinimalCollisionData = false;

	//use collision detection on the present camera!
	bool hit = getCamera(camera_name)->selectWorld(
										view_x,
										view_y,
										window_width,
										window_height,
										selectionRecorder,
										defaultSettings);
	if(hit) {
		cVector3d pos = selectionRecorder.m_nearestCollision.m_localPos;
		auto object = selectionRecorder.m_nearestCollision.m_object;
		if(object->getParent() == NULL) {
			return false;
		}
		auto object_parent = object->getParent();
		bool f_found_parent_link = false;
		cTransform transform = object->getLocalTransform();
		cRobotLink* link;
		while (object_parent != NULL) {
			if (robot_name == object_parent->m_name) {
				return true;
			}
			if (!f_found_parent_link) {
				// try casting to cRobotLink
				link = dynamic_cast<cRobotLink*> (object_parent);
				if (link != NULL) {
					f_found_parent_link = true;
					ret_link_name = link->m_name;
					// position is with respect to the graphic object. need to go up to the link frame
					pos = transform * pos;
					ret_pos << pos.x(), pos.y(), pos.z();
				} else {
					transform = object_parent->getLocalTransform()*transform;
				}
			}
			object_parent = object_parent->getParent();
		}
	}
	return false;
}

// get camera object
cCamera* Sai2Graphics::getCamera(const std::string& camera_name) {
	cCamera* camera;
	// find camera among children
	// TODO: this can be optimized by locally storing list of cameras
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		cGenericObject* child = _world->getChild(i);
		if (camera_name == child->m_name) {
			camera = dynamic_cast<cCamera*>(child);
			if (camera) {
				break;
			}
		}
	}
	if (!camera) {
		cerr << "Could not find camera named " << camera_name << endl;
		abort();
		//TODO: throw exception instead
	}
	return camera;
}

cRobotLink* Sai2Graphics::findLinkRecursive(cRobotLink* parent, const std::string& link_name) {
	// call on children
	cRobotLink* child;
	cRobotLink* ret_link = NULL;
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(parent->getChild(i));
		if (child != NULL) {
			if (child->m_name == link_name) {
				ret_link = child;
				break;
			} else {
				ret_link = findLinkRecursive(child, link_name);
			}
		}
	}
	return ret_link;
}

cRobotLink* Sai2Graphics::findLink(const std::string& robot_name, const std::string& link_name) {
	// get robot base
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}

	cRobotLink* target_link = NULL;
	// get target link
	cRobotLink* base_link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (base_link != NULL) {
			if (base_link->m_name == link_name) {
				target_link = base_link;
				break;
			} else {
				target_link = findLinkRecursive(base_link, link_name);
			}
		}
	}
	return target_link;
}

// Show frame for a particular link or all links on a robot.
bool Sai2Graphics::showLinkFrame(bool show_frame,
                         			const std::string& robot_name,
                         			const std::string& link_name,
                         			const double frame_pointer_length) {
	bool fShouldApplyAllLinks = false;
	if (link_name.empty()) {
		fShouldApplyAllLinks = true;
	}

	// get robot base
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}

	// apply frame show
	if (fShouldApplyAllLinks) {
		// TODO: apply to whole robot
	} else {
		cRobotLink* target_link = NULL;
		// get target link
		cRobotLink* base_link;
		for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
			base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
			if (base_link != NULL) {
				if (base_link->m_name == link_name) {
					target_link = base_link;
					break;
				} else {
					target_link = findLinkRecursive(base_link, link_name);
				}
			}
		}
		if (target_link == NULL) {
			cerr << "Could not find link in robot: " << link_name << endl;
			abort();
		}

		// set wireframe whenever we show frame
		target_link->setWireMode(show_frame, true);
		target_link->setFrameSize(frame_pointer_length, false);
		target_link->setShowFrame(show_frame, false);
	}
	return show_frame; // TODO: handle correctly
}

}
