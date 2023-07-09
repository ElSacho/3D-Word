#include "scene.hpp"


using namespace cgp;


void scene_structure::update_camera()
{
	inputs_keyboard_parameters const& keyboard = inputs.keyboard;

	// The camera moves forward all the time
	//   We consider in this example a constant velocity, so the displacement is: velocity * dt * front-camera-vector
	float const dt = timer.update();

	// The camera rotates if we press on the arrow keys
	//  The rotation is only applied to the roll and pitch degrees of freedom.
	float const roll  = 30.0f; // speed of the roll
	if (keyboard.left)
		environment.camera.manipulator_translate_in_plane(vec2{dt*roll,  0});
	if (keyboard.right)
		environment.camera.manipulator_translate_in_plane(vec2{-dt*roll,  0});
	if (keyboard.down)
		environment.camera.manipulator_translate_in_plane(vec2{0,  dt*roll});
	if (keyboard.up)
		environment.camera.manipulator_translate_in_plane(vec2{0,  -dt*roll});
}

void scene_structure::initialize()
{
	//inputs;
	//skybox initialization (path contains 6 images (cube), with a certain order)
	skybox.initialize("assets/skybox_4/"); 
	
	
	// Basic set-up
	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.axis = camera_spherical_coordinates_axis::z;
	environment.camera.look_at({ 0.0f,2.0f,150.0f }, { 0,0,100 });

	//On initialise le train
	piston1.initialize();
	piston1.initialiserWagon();

	//On initialise la montagne
	mont.initialize();

	// On initialise les flocons
	flocon1.initialize();
	perlin_noise_parameters parameters;
	
	//On s'occupe des points de passage de notre Orient Express
	buffer<vec3> key_positions =
	{ {-300, 85,0}, {-150, 85,0}, {-120, 65,0}, {-100, 55,0}, {-60, 45,0}, {-30, 40,0}, {0, 25,0}, {20, 18,0}, {30, 16,0}, {80, -3,0}, {115, -11,0}, {135, -22,0}, {170, -27,0}, {200, -37,0}, {230, -47,0}, {266, -48,0}, {300, -49,0}, {320, -54,0}, {380, -65,0} };
	for(int i=0;i<18;++i){
		float temp = key_positions[i][1];
		key_positions[i][1]=key_positions[i][0];
		key_positions[i][0]=temp;
	}
	for(int i=0;i<18;++i){
		key_positions[i][2]=evaluate_terrain_height_perlin(key_positions[i][0]/mont.terrain_longueur+0.5f,key_positions[i][1]/mont.terrain_longueur+0.5f, parameters);
	}
	buffer<float> key_times = 
	{ 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0F, 14.0f, 15.0f, 16.0f, 17.0f , 18.0f};
	buffer<vec3> key_positions2=key_positions;
	int N = key_times.size();
	timer.t_min = key_times[1];
	timer.t_max = key_times[N - 2];
	timer.t = timer.t_min;
	buffer<vec3> key_positions3=key_positions;
	for(int i=0;i<18;++i){
		key_positions3[i][1]=0;
	}


	//On initilise les rails jusquà une certaine hauteur
	float hauteur = 3.0f;
	chemin1.createAllRail(1.0f, 0.1f, mont.terrain_mesh,key_positions,key_times, timer.t_min, timer.t_max, hauteur);
	chemin1.key_positions2 = key_positions2;
	chemin1.key_positions3 = key_positions3;
	
	
	// On initialise les positions initiales du train et des informations sur leur vitesse et le tableau de conversion
	// Le tableau de conversion sert a convertir la distance theorique de la fonction interpolation avec la distance reelement parcourue
	tabConversion = convertirEnVraieDist(key_positions, key_times,  mont.terrain_mesh,timer.t_min, timer.t_max);
	float dIni = timer.t_min+2.5f;
	float conv = findConversionToVraieDist(tabConversion, dIni);
	piston1.initializeValue(dIni,conv);
	mouvement = {0.1,0.1, dIni ,0,0,conv,conv};
	mouvementWagon = {timer.t_min+0.1f,0,0};
	mouvementWagon2 = {0,0};
	mouvementWagon3 = {0,0};
	mouvementWagon4 = {0,0};
	mouvementWagon5 = {0,0};
	mouvementWagon6 = {0,0};
	mouvementWagon7 = {0,0};
	ecartEntreWagon = 9.0f;
	vitesseMax = 5.0f;



	// On initialise les igloos et le pecheur
	vecIgloo = generateIgloo(20,  generate_positions_on_lac(20, mont));
	positionInitiale = vec3{key_positions[1][0]-10.0,key_positions[1][1],100}+vec3{-0.15,-0.4,1.4};
	igloo1.initializeFil(positionInitiale);
	igloo1.initializePeche({key_positions[1][0]-10.0,key_positions[1][1],100});

	// Mouche
	nb_follower_mouches = 15;
	follower_mouches;
	speeds_mouches;
    leader.initialize_leader_mouche(positionInitiale);
	//On initialise les mouches selon un trace circulaire autour du pecheur
	for (int i = 0; i < nb_follower_mouches; i++) {
		follower_mouches.push_back(leader.key_positions[1] + 1.0f*vec3(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)), static_cast <float> (rand()) / (static_cast <float> (RAND_MAX)), static_cast <float> (rand()) / (static_cast <float> (RAND_MAX))));
		speeds_mouches.push_back({ 0.1f, 0.1f, 0.1f });
	}
	//vitesse du leader
	speeds_mouches.push_back({ 0.1f, 0.1f, 0.1f });


}


void scene_structure::display()
{

	//On affiche la skybox
	draw(skybox, environment); 

	// Basic elements of the scene
	environment.light = environment.camera.position();
	if (gui.display_frame){
		draw(global_frame, environment);
	}

	//On affiche la montagne et le rail
	draw(mont.terrain_drawable, environment);
	draw(mont.lac_drawable, environment);
	draw(chemin1.hierarchy_rail, environment); 

	// On affiche les igloos
	draw(trouPoisson, environment);
	display_igloo(vecIgloo, environment);

	if (gui.display_wireframe){
		draw_wireframe(mont.terrain_drawable, environment);
	}
	timer_train;
	//On met a jour les timers
	// Un timer fait une boucle quand l 'autre ne fait pas de boucle
	timer2.update();
	timer.update();
	leader.timer3.update();
	float t = timer.t;
	float t2 = timer2.t;

	//On met a jour et affiche les flocons et la fumee
	//flocon1.update();
	//flocon1.display_semiTransparent(environment);


	//On met a jour les positions du train et affiche les wagons
	vec3 p = interpolation(mouvement[2], chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvement = piston1.acceleration2(mouvement[0], mouvement[1], vitesseMax, 0.04f, mont.terrain_mesh, p, chemin1, tabConversion, timer.t_min, timer.t_max,mouvement[3],mouvement[4],mouvement[5],mouvement[6]);
	//mouvement = piston1.acceleration(mouvement[0], mouvement[1], vitesseMax, 0.04f, mont.terrain_mesh, p, chemin1, mouvement[2], timer.t_min, timer.t_max,mouvement[3],mouvement[4],mouvement[5]);
	float dw = findConversion(tabConversion, mouvement[5]-7.0f);
	vec3 pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon2 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0f,timer.t_min, timer.t_max, mouvementWagon2[0],mouvementWagon2[1], tabConversion);
	draw(piston1.wagon1, environment);
	dw = findConversion(tabConversion, mouvement[5]-7.0f-ecartEntreWagon);
	pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon3 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0-ecartEntreWagon,timer.t_min, timer.t_max, mouvementWagon3[0],mouvementWagon3[1], tabConversion);
	draw(piston1.wagon1, environment);
	dw = findConversion(tabConversion, mouvement[5]-7.0f-ecartEntreWagon*2);
	pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon4 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0-2*ecartEntreWagon,timer.t_min, timer.t_max, mouvementWagon4[0],mouvementWagon5[1], tabConversion);
	draw(piston1.wagon1, environment);
	dw = findConversion(tabConversion, mouvement[5]-7.0f-ecartEntreWagon*3);
	pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon5 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0-3*ecartEntreWagon,timer.t_min, timer.t_max, mouvementWagon5[0],mouvementWagon5[1], tabConversion);
	draw(piston1.wagon1, environment);
	dw = findConversion(tabConversion, mouvement[5]-7.0f-ecartEntreWagon*4);
	pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon6 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0-4*ecartEntreWagon,timer.t_min, timer.t_max, mouvementWagon6[0],mouvementWagon6[1], tabConversion);
	draw(piston1.wagon1, environment);
	dw = findConversion(tabConversion, mouvement[5]-7.0f-ecartEntreWagon*5);
	pw = interpolation(dw-0.01, chemin1.key_positions, chemin1.key_times, mont.terrain_mesh);
	mouvementWagon7 = piston1.updateWagon2(mouvement[1],0.04f, mont.terrain_mesh, pw, chemin1,mouvement[5]-7.0-5*ecartEntreWagon,timer.t_min, timer.t_max, mouvementWagon7[0],mouvementWagon7[1], tabConversion);
	draw(piston1.wagon1, environment);
	//On affiche la locomotive
	draw(piston1.hierarchy, environment);



	//On affiche le pecheur
	displayPeche(igloo1, environment,t2);
	igloo1.displayFil(positionInitiale, timer2, environment);

	//On affiche les mouches autour du pecheur
	leader.update_leader_mouche(leader.timer3, 0.04f, speeds_mouches);
	for (int i = 0; i < 20; i++) {
		leader.update_follower_mouches(follower_mouches, speeds_mouches, t2, 0.04f, 0.0001f, 0.0001f, 0.005f);
	}
	for (cgp::vec3 pos : follower_mouches) {
		leader.miseAJour(pos);
		draw(leader.hierarchy, environment);
    }

}




void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::Checkbox("Wireframe", &gui.display_wireframe);

}


