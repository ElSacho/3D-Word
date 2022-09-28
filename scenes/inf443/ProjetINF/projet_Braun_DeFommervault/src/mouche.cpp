#include "mouche.hpp"
#include "interpolation.hpp"
#include <cmath>
#include <list> 

using namespace cgp;



int idx_last_key_time;


void mouche::initialize_mouche(cgp::vec3 position){

	key_times;
	this->changePosition(position);
	int N = key_times.size();
	timer3.t_min = key_times[1]+0.1f;
	timer3.t_max = key_times[N - 2]-0.1f;
	timer3.t = timer3.t_min;
	// Create the hierarchy
	// ************************************ //
	mesh base_mesh = mesh_primitive_sphere(0.01f);
	base.initialize(base_mesh, "base");
	mesh body_mesh = mesh_primitive_sphere(0.5f);
	body.initialize(body_mesh, "body");
	body.anisotropic_scale = vec3{2.0f, 1, 1};
	boule_tete.initialize(mesh_primitive_sphere(0.4f), "head");
	aile_gauche_1.initialize(mesh_primitive_quadrangle(vec3{0, 0, 0}, vec3{1, 0, 0}, vec3{1, 1, 0}, vec3{0, 1, 0}), "Wing 1 L");
	aile_gauche_2.initialize(mesh_primitive_quadrangle(vec3{0, 0, 0}, vec3{1, 0, 0}, vec3{0.2f, 1, 0}, vec3{0, 1, 0}), "Wing 2 L");
	eye1.initialize(mesh_primitive_sphere(0.05f), "eye 1");
	eye2.initialize(mesh_primitive_sphere(0.05f), "eye 2");
	noze.initialize(mesh_primitive_cone(0.15f, 0.25f), "noze"); noze.transform.rotation = rotation_transform::from_axis_angle({ 0,1,0 }, 8*3.14f/14.0f);
	aile_droite_1.initialize(mesh_primitive_quadrangle(vec3{0, 0, 0}, vec3{1, 0, 0}, vec3{1, -1, 0}, vec3{0, -1, 0}), "Wing 1 R");
	aile_droite_2.initialize(mesh_primitive_quadrangle(vec3{0, 0, 0}, vec3{1, 0, 0}, vec3{0.2f, -1, 0}, vec3{0, -1, 0}), "Wing 2 R");
	eye1_01.initialize(mesh_primitive_sphere(0.02f), "eye 1 01");
	eye1_02.initialize(mesh_primitive_sphere(0.02f), "eye 1 02");


	// ajout des textures et couleurs
	//colors
	//train.texture = opengl_load_texture_image("assets/neige_train01.jpeg");
	base.shading.color = { 0.70f, 0.27f, 0.08f };
	base.shading.phong.ambient = 0.28;
	base.shading.phong.diffuse = 1.0;
	base.shading.phong.specular = 0.5;
	base.shading.phong.specular_exponent = 12.8;

	float alpha;
	float ambient;
	float diffuse;
	float specular;
	float c1;
	float c2;
	float c3;

	c1=0.0;
	c2=0.0;
	c3=0.0;
	ambient = 0.28;
	diffuse =1.0;
	specular = 0.5;
	alpha = 12.8;

	body.shading.color = { c1,c2,c3 };
	boule_tete.shading.color = { c1,c2,c3 };
	aile_droite_1.shading.color = { c1,c2,c3 };
	aile_droite_2.shading.color = { c1,c2,c3 };
	aile_gauche_1.shading.color = { c1,c2,c3 };
	aile_gauche_2.shading.color = { c1,c2,c3 };






		// color

	vec3 color1 = { 0, 0, 0 };
	vec3 color2 = { 1, 0.5f, 0 };
	eye1.shading.color = color1;
	eye2.shading.color = color1;
	noze.shading.color = color2;

	// ajout de la hierarchie

	
    hierarchy.add(base);

	//hierarchie piston 1
	hierarchy.add(body, "base",{ 0,0,0 });
	hierarchy["base"].transform.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, Pi /2);
	hierarchy.add(boule_tete, "body", { 1.0f ,0,0.2f });
	hierarchy.add(aile_gauche_1, "body", { -0.5f,0.15,0 }); 
	hierarchy.add(aile_gauche_2, "Wing 1 L", { 0,1,0 });
	hierarchy.add(eye1, "head", {0.36f, 0.10, 0.10});
	hierarchy.add(eye2, "head", {0.36f, -0.10, 0.10});
	hierarchy.add(noze, "head", {0.32f, 0, -0.10f});
	hierarchy.add(aile_droite_1, "body", { -0.5f,0.15,0 }); 
	hierarchy.add(aile_droite_2, "Wing 1 R", { 0,-1,0 });
	hierarchy.add(eye1_01, "eye 1", { 0.04f,0,0 });
	hierarchy.add(eye1_02, "eye 2", { 0.04f,0,0 });

	hierarchy["base"].transform.scaling=0.01f;

}

void mouche::changePosition(cgp::vec3 position){
	int N = this->key_times.size();
	key_positions;
	for(int i=0;i<N;++i){
		key_positions.push_back(vec3{std::cos(i*2*Pi/N),std::sin(i*2*Pi/N),std::sin(i)/4}+position);

	}
}


void mouche::initialize_leader_mouche(cgp::vec3 position)
{
	//On place l oiseau leader en position une
	this->initialize_mouche(position);
	std::cout<< this->hierarchy["base"].transform.translation <<std::endl;
	std::cout<< "zfcnefcvcevcscezcvez paeiouvchoepau^hvc" <<std::endl;
	key_positions;
	key_times;
	idx_last_key_time = 1;
	//this->hierarchy["base"].transform.translation = key_positions[1];
	//this->hierarchy["base"].transform.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, std::asin((key_positions[2][0] - key_positions[1][0]) / norm((key_positions[2] - key_positions[1]))));
	this->hierarchy.update_local_to_global_coordinates();
}


void mouche::update_mouche(cgp::vec3 position, float t)
{
	/** *************************************************************  **/
	/** Compute the (animated) transformations applied to the elements **/
	/** *************************************************************  **/

	this->hierarchy["base"].transform.translation = position;

	hierarchy["head"].transform.rotation = rotation_transform::from_axis_angle({ 0,1,0 }, std::sin(t*2)/5);
	hierarchy["Wing 1 L"].transform.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, std::sin(t*5)/2);
	hierarchy["Wing 2 L"].transform.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, std::sin(t*5)/1.5f);
	hierarchy["Wing 1 R"].transform.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, -std::sin(t*5)/2);
	hierarchy["Wing 2 R"].transform.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, -std::sin(t*5)/1.5f);
	hierarchy["eye 1"].transform.rotation = rotation_transform::from_axis_angle({ 0,1,0 }, std::sin(t*2)/5);
	hierarchy["eye 2"].transform.rotation = rotation_transform::from_axis_angle({ 0,1,0 }, std::sin(t*2)/5);
	// update the global coordinates
	this->hierarchy.update_local_to_global_coordinates();
}


void mouche::update_leader_mouche(cgp::timer_interval timer3, float dt, cgp::buffer<cgp::vec3> &vecVitesse) {
	// INTERPOLATION
	// Compute the interpolated position
	vec3 const p = interpolationSansGlace(timer3.t, key_positions, key_times);
	vecVitesse[vecVitesse.size() - 1] = (p - this->hierarchy["base"].transform.translation) / dt;
	// Compute the orientation
	/*
	int N_t = key_times.size() - 2;
	// Si on est encore dans la trajectiore on continue dedans
	if (idx_last_key_time < N_t && ((int) timer3.t) % ((int) key_times[N_t]) > key_times[idx_last_key_time + 1]) {
		int next = idx_last_key_time + 2;
		idx_last_key_time++;
		std::cout<< idx_last_key_time <<std::endl;	

		std::cout<< idx_last_key_time <<std::endl;	
	}
	//Sinon on recommence au debut
	if (idx_last_key_time >= N_t - 2) {
		idx_last_key_time = 1;
	}
	*/
	this->update_mouche( p, timer3.t);
}


void mouche::update_follower_mouches(cgp::buffer<cgp::vec3> &followers, cgp::buffer<cgp::vec3> &vecVitesse, float t, float dt, float k_attr, float k_rep, float k_frott)
{
	vec3 force, direction;
	float dist;
	const float distMax = 1.0f;
	int nb = followers.size();
	for (int i = 0; i < nb; i++) {
		force  = { 0, 0, 0 };
		for (int j = 0; j < nb; j++) {
			if (j == i) continue;
			direction = followers[j] - followers[i];
			dist = std::sqrt((direction[0])*(direction[0])+(direction[1])*(direction[1])+(direction[2])*(direction[2]));
			direction /= dist;
			force += - k_rep * direction / (dist * dist) / nb - k_frott * (vecVitesse[i] - vecVitesse[j]) / nb;
		}
		direction = this->hierarchy["base"].transform.translation - followers[i];
		dist = std::sqrt((direction[0])*(direction[0])+(direction[1])*(direction[1])+(direction[2])*(direction[2]));
		direction /= dist;
		force += 1000000*k_attr * dist * dist * direction - k_rep * direction / (dist * dist) - k_frott * (vecVitesse[i] - vecVitesse[nb]);
		vecVitesse[i] += dt * force;
		followers[i] = followers[i] + dt * vecVitesse[i];
		direction = this->hierarchy["base"].transform.translation - followers[i];
		dist = std::sqrt((direction[0])*(direction[0])+(direction[1])*(direction[1])+(direction[2])*(direction[2]));
		if (dist > distMax && dot(vecVitesse[i], vecVitesse[nb]) < 0)
			vecVitesse[i] /= std::sqrt((vecVitesse[i][0])*(vecVitesse[i][0])+(vecVitesse[i][1])*(vecVitesse[i][1])+(vecVitesse[i][2])*(vecVitesse[i][2]));
	}
}


void mouche::miseAJour(cgp::vec3 pos){
	this->hierarchy["base"].transform.translation = pos;
	this->hierarchy.update_local_to_global_coordinates();
}
