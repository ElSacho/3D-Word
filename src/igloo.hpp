#pragma once

#include "cgp/cgp.hpp"
#include "models_textures.hpp"
#include "terrain2.hpp"

#include <list> 


struct igloo{

    cgp::hierarchy_mesh_drawable hierarchy_igloo;
    cgp::mesh_drawable mur;
    cgp::mesh_drawable porte;


    cgp::mesh_drawable trouPoisson;
    cgp::mesh_drawable fond;
    montagne fondEau;
    cgp::hierarchy_mesh_drawable hierachy_trou;

    cgp::mesh_drawable bras;          
	cgp::mesh_drawable corps;    


    //peche
    std::array<cgp::vec3, 25> p;
	std::array<cgp::vec3, 25> v;
	std::array<cgp::vec3, 25> f;

    void simulation_step(float dt);
	void draw_segment(cgp::vec3 const& a, cgp::vec3 const& b, const cgp::scene_environment_basic_camera_spherical_coords environment2);
    void displayFil(cgp::vec3 position, cgp::timer_basic timer, const cgp::scene_environment_basic_camera_spherical_coords environment2);

	// Drawable structure to display the fil de peche
	cgp::mesh_drawable particle_sphere;
	cgp::segments_drawable segment;

	// Timer used for the animation
	cgp::timer_basic timer;
    float L0; // Rest-length of spring
    perlin_noise_parameters parameters;

    void initializeFil(cgp::vec3 position);
    void initializeAction(cgp::vec3 position);
	void initializeUnIgloo(float h, float l, float taux);
    void initializePeche(cgp::vec3 position);

};

void displayPeche(igloo peche1, const cgp::scene_environment_basic_camera_spherical_coords environment2,float t);
std::list<igloo> generateIgloo(int N, std::vector<cgp::vec3> positions);
void display_igloo(std::list<igloo> vecIgloo, const cgp::scene_environment_basic_camera_spherical_coords environment2);

