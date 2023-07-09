#pragma once

#include "cgp/cgp.hpp"
#include <list> // utilise pour le stockage des positions - vitesses des flocons

struct particle_structure
{
	cgp::vec3 p; // Position
	cgp::vec3 v; // Speed
	//float scale;
};



struct flocon
{
    // flocons
	std::list<particle_structure> particles; // Storage of all currently active particles
	cgp::mesh_drawable sphere;

    // billboards flocons
	cgp::mesh_drawable quad_1;

	// Timer used for the interpolation of the position
	cgp::timer_event_periodic timer;

	const cgp::scene_environment_basic_camera_spherical_coords environment2; // Standard environment controler

    void initialize();
    void update();

	//template <typename SCENE_ENVIRONMENT>
    // void display_semiTransparent(SCENE_ENVIRONMENT const& environment);
   void display_semiTransparent(const cgp::scene_environment_basic_camera_spherical_coords environment2);
};


