#pragma once

#include "cgp/cgp.hpp"
#include "models_textures.hpp"


struct mouche {


	cgp::hierarchy_mesh_drawable hierarchy;
	cgp::timer_interval timer3;



	cgp::mesh_drawable body;
	cgp::mesh_drawable base;
	cgp::mesh_drawable boule_tete;
	cgp::mesh_drawable aile_gauche_1;
	cgp::mesh_drawable aile_gauche_2;
	cgp::mesh_drawable aile_droite_1;
	cgp::mesh_drawable aile_droite_2;
	cgp::mesh_drawable eye1;
	cgp::mesh_drawable eye2;
	cgp::mesh_drawable noze;
	cgp::mesh_drawable eye1_01;
	cgp::mesh_drawable eye1_02;


	cgp::buffer<cgp::vec3> key_positions;
	cgp::buffer<float> key_times = { 0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 26.0f, 28.0f, 30.0f, 32.0f, 34.0f, 36.0f, 38.0f };

	int idx_last_key_time;

	void initialize_mouche(cgp::vec3 position);
	void changePosition(cgp::vec3 position);
	void initialize_leader_mouche(cgp::vec3 position);
	void update_mouche(cgp::vec3 position, float t);
	void update_leader_mouche(cgp::timer_interval timer3, float dt, cgp::buffer<cgp::vec3>& speeds);
	void update_follower_mouches(cgp::buffer<cgp::vec3>& followers, cgp::buffer<cgp::vec3>& speeds, float t, float dt, float k_attr, float k_rep, float k_frott);
	void miseAJour(cgp::vec3 pos);

};
