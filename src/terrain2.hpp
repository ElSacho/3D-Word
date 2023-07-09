#pragma once

#include "cgp/cgp.hpp"

struct perlin_noise_parameters
{
	float persistency = 0.574f;
	float frequency_gain = 1.515f;
	int octave = 15;
	float terrain_height = 100.4f;
};



struct montagne{
	// Initialize the mesh of the terrain
	void create_terrain_mesh3(float longueur);  
	cgp::mesh create_terrain_mesh(float longueur);    
  
	

	// points de controle
	cgp::mesh_drawable sphere2;
	std::array<cgp::vec2, 18> pts_controle;

	// montagne
	cgp::mesh terrain_mesh;
	cgp::mesh_drawable terrain_drawable;
	perlin_noise_parameters parameters;
	float terrain_longueur;

	//lac
	cgp::mesh lac_mesh;
	cgp::mesh_drawable lac_drawable;
	float hauteurGlace;

	// donne l'altitude d'un point de coordonnees (x,y) 

	// Recompute the vertices of the terrain everytime a parameter is modified
	//  and update the mesh_drawable accordingly
	void update_terrain(cgp::mesh& terrain, cgp::mesh_drawable& terrain_visual, perlin_noise_parameters const& parameters);
	void initialize();
	cgp::mesh create_terrain_mesh2(int N, float terrain_length,  perlin_noise_parameters const& parameters);

};

float evaluate_terrain_height_perlin(float u, float v, perlin_noise_parameters const& parameters);
float evaluate_terrain_height(float x, float y, cgp::mesh& terrain);
std::vector<cgp::vec3> generate_positions_on_lac(int N, montagne terrain);
