#pragma once


#include "cgp/cgp.hpp"
#include "models_textures.hpp"
#include "interpolation.hpp"





//struct gui_parameters {
//	bool display_frame = true;
//	bool display_wireframe = false;
//};

// The element of the GUI that are not already stored in other structures

struct piston{
	//objets piston
    cgp::mesh_drawable train;
    cgp::mesh_drawable wheel1;
    cgp::mesh_drawable wheel2;
    cgp::mesh_drawable wheel3;
	cgp::mesh_drawable wheel1_2;
    cgp::mesh_drawable wheel2_2;
    cgp::mesh_drawable wheel3_2;
	cgp::mesh_drawable base1;
	cgp::mesh_drawable base2;


	cgp::mesh_drawable global_frame;          // The standard global frame
	cgp::scene_environment_basic_camera_spherical_coords environment; // Standard environment controler
	cgp::inputs_interaction_parameters inputs; // Storage for inputs status (mouse, keyboard, window dimension)
	cgp::hierarchy_mesh_drawable hierarchy;
	cgp::rotation_transform rot;


	// pieces de la locomotive
	cgp::mesh_drawable biele3;
	cgp::mesh_drawable biele2;
	cgp::mesh_drawable vis1;
	cgp::mesh_drawable vis2;
	cgp::mesh_drawable vis3;

	cgp::mesh_drawable biele3_2;
	cgp::mesh_drawable biele2_2;
	cgp::mesh_drawable vis1_2;
	cgp::mesh_drawable vis2_2;
	cgp::mesh_drawable vis3_2;
	void initialize();
	using environmentType = cgp::scene_environment_basic_camera_spherical_coords;

	//Pour les wagons
	cgp::mesh_drawable wagon1;


	float dinitiale;
	float convertie;

	void initialiserWagon();
	cgp::vec3 updateWagon(float vitesse, float dt, cgp::mesh& terrain, cgp::vec3 position, chemin chemin1, float distance, float tMin, float tMax, float degX, float degZ);
	cgp::vec2 updateWagon2(float vitesse, float dt, cgp::mesh& terrain, cgp::vec3 position, chemin chemin1, float vraieDist, float tMin, float tMax, float degX, float degZ, cgp::buffer<cgp::vec2> tabConversion);


	//cgp::hierarchy_mesh_drawable hierarchy;
    void tournerRoue(float t);
	void avancer(float vitesse);
	void translationPiston(cgp::vec3 p);
	void update(cgp::vec3 const& p, environmentType& environment, float hauteur);
	cgp::buffer<float> acceleration(float accelerationAvant, float vitesse, float vitesseMax, float dt, cgp::mesh& terrain, cgp::vec3 position, chemin chemin1, float distance, float tMin, float tMax, float degX, float degZ,float vraieDist);
	cgp::buffer<float> acceleration2(float accelerationAvant, float vitesse, float vitesseMax, float dt, cgp::mesh& terrain, cgp::vec3 position, chemin chemin1, cgp::buffer<cgp::vec2> tabConversion, float tMin, float tMax, float degX, float degZ,float vraieDist, float vraieDist2);
	void initializeValue(float dini, float conv);
};


