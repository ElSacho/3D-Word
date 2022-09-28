#include "igloo.hpp"

using namespace cgp;

void igloo::initializeUnIgloo(float h, float l, float taux){

    mesh mesh_mur = mesh_primitive_ellipsoid({h*1.4,h*taux,h}, {0,0,0}, 10, 10);
    mesh mesh_porte = cylinder_with_hole(h/2, h/2-0.4, h/4);
    mesh_porte.position+={0,0,h};
    mesh_mur.push_back(mesh_porte);
    mur.initialize(mesh_mur,"mur");

    mur.transform.rotation= rotation_transform::from_axis_angle({ 0,1,0 },Pi/2);

    GLuint const texture_igloo_id = opengl_load_texture_image("assets/igloo.jpeg",
		GL_CLAMP_TO_BORDER,
		GL_CLAMP_TO_BORDER);
	
	mur.texture = texture_igloo_id;

}

std::list<igloo> generateIgloo(int N, std::vector<cgp::vec3> positions){
    std::list<igloo> listeIgloo;
    for (vec3& position : positions){
        float h = rand_interval(1.5, 3.0);
        float taux = rand_interval(1, 2);
        igloo igloo1;
        igloo1.initializeUnIgloo( h, h/2, taux);
        igloo1.mur.transform.translation=position;
        listeIgloo.push_back(igloo1);
    }
    std::cout<< "je suis ici" <<std::endl;
    return listeIgloo;
}

void display_igloo(std::list<igloo> vecIgloo, const cgp::scene_environment_basic_camera_spherical_coords environment2){
    for (igloo igloo1 : vecIgloo){
        draw(igloo1.mur, environment2);
    }
}


void igloo::initializePeche(cgp::vec3 position){
    //trou de peche
    trouPoisson.initialize(trouPourPoisson(5.0, 0.5, 0.2),"trou poisson");  
    fond.initialize(disc_with_texture(0.5),"eau");
	float l = 1.0;
	fondEau.create_terrain_mesh3(l);


    GLuint const texture_lac = opengl_load_texture_image("assets/lac.jpeg",
		GL_CLAMP_TO_BORDER,
		GL_CLAMP_TO_BORDER);
	
	trouPoisson.texture = texture_lac;

    fond.shading.color={1,1,1};

    trouPoisson.transform.translation = position+vec3{0,-0.6,0};
    fond.transform.translation = position+vec3{0,-0.6,0};
	fondEau.terrain_mesh.position+=position+vec3{0,-0.6,0};
	fondEau.lac_drawable.initialize(fondEau.terrain_mesh,"eau");

 

    //pecheur
   	mesh brasM = mesh_load_file_obj("objets/bras02.obj");
	mesh corpsM = mesh_load_file_obj("objets/corps02.obj");
	corps.initialize(corpsM, "corps");
	bras.initialize(brasM, "bras");
    corps.transform.translation = position;
    corps.transform.rotation=rotation_transform::from_axis_angle({ 1,0,0 },Pi/2);
    bras.transform.translation=vec3{-0.15,0,1.4}+position;
   
    GLuint const texture_pecheur = opengl_load_texture_image("assets/pecheur6.jpg",
		GL_CLAMP_TO_BORDER,
		GL_CLAMP_TO_BORDER);
	
	corps.texture = texture_pecheur;

}


void displayPeche(igloo peche1, const cgp::scene_environment_basic_camera_spherical_coords environment2, float t){
    peche1.bras.transform.rotation=rotation_transform::from_axis_angle({ 1,0,0 },std::sin(t));
	perlin_noise_parameters parameters;
	parameters.frequency_gain = 2250.0+std::sin(t)*249;
	parameters.octave=8;
	parameters.persistency=0.6f;
	parameters.terrain_height = 0.1f;
    draw(peche1.corps,environment2);
    draw(peche1.bras,environment2);
    draw(peche1.trouPoisson,environment2);
    draw(peche1.fond,environment2);
}

int N = 25;

// Spring force applied on particle p_i with respect to position p_j.
vec3 spring_force(vec3 const& p_i, vec3 const& p_j, float L0, float K)
{
	vec3 const p = p_i - p_j;
	float const L = norm(p);
	vec3 const u = p / L;

	vec3 const F = -K * (L - L0) * u;
	return F;
}


void igloo::simulation_step(float dt)
{
	// Simulation parameters
	float m = 0.01f;       // particle mass
	float K = 10.0f;        // spring stiffness
	float mu = 0.005f;      // damping coefficient

	vec3 g = { 0,0,-9.81f }; // gravity

	
	
	for (int i=1; i<N; i++){
		// Forces
		vec3 f_spring;
		if (i==N-1){
			f_spring = spring_force(p[i], p[i-1], L0, K);
		}
		else{
			f_spring = spring_force(p[i], p[i-1], L0, K) + spring_force(p[i], p[i+1], L0, K);
		}
		vec3 f_weight = m * g;
		vec3 f_damping = -mu * v[i];
		f[i] = f_spring + f_weight + f_damping;
	}


	for (int i=1; i<N; i++){
			//integration numerique
			v[i] = v[i] + dt * f[i] / m;
			p[i] = p[i] + dt * v[i];
	}

}


void igloo::draw_segment(vec3 const& a, vec3 const& b, const cgp::scene_environment_basic_camera_spherical_coords environment2)
{
	segment.update({ a, b });
	draw(segment, environment2);
}

void igloo::initializeFil(cgp::vec3 position)
{
	for (int i=0; i<N; i++){
		float y = i;
		p[i] = vec3{0.0f, 0.0f, -4.0f*y/N}+position;
		v[i] = {0.0f, 0.0f, 0.0f};
	}
	
	L0 = 1.0f/N;

	particle_sphere.initialize(mesh_primitive_sphere(0.05f));
	segments_drawable::default_shader = curve_drawable::default_shader;
	segment.initialize({{0,0,0},{1,0,0}});

}





void igloo::displayFil(cgp::vec3 position, cgp::timer_basic timer, const cgp::scene_environment_basic_camera_spherical_coords environment2)
{
	// Update the current time
	timer.update();
	float t = timer.t;

for (int j = 0; j<3;j++){
	simulation_step(timer.scale * 0.003f);
}
	p[0]=position+vec3{0.0,-0.6*std::cos(std::sin(t))+0.35,-0.6*std::sin(std::sin(t))};

	for (int i=0; i<N; i++){
		particle_sphere.transform.translation = p[i];
		if (i>0){
			draw_segment(p[i-1], p[i],environment2);
		}
	}

}

void igloo::initializeAction(cgp::vec3 position){
    this->initializeFil(position);
    this->initializePeche(position);


}
