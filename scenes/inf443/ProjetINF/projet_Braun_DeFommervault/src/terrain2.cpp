
#include "terrain2.hpp"

using namespace cgp;

mesh montagne::create_terrain_mesh(float longueur)
{
	//cree un terrain plat
	int const terrain_sample = 1000;
	float l = longueur/2;
	mesh terrain = mesh_primitive_grid({-l,-l,0},{l,-l,0},{l,l,0},{-l,l,0},terrain_sample,terrain_sample);
	return terrain;
}

void montagne::create_terrain_mesh3(float longueur)
{
	//cree un terrain plat
	int const terrain_sample = 1000;
	float l = longueur/2;
	terrain_mesh = mesh_primitive_grid({-l,-l,0},{l,-l,0},{l,l,0},{-l,l,0},terrain_sample,terrain_sample);
}

void montagne::update_terrain(mesh& terrain, mesh_drawable& terrain_visual, perlin_noise_parameters const& parameters)
{
	// Number of samples in each direction (assuming a square grid)
	int const N = std::sqrt(terrain.position.size());

	// Recompute the new vertices
	for (int ku = 0; ku < N; ++ku) {
		for (int kv = 0; kv < N; ++kv) {
			
			// Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);

			int const idx = ku*N+kv;

			// Compute the Perlin noise
			float const noise = noise_perlin({u, v}, parameters.octave, parameters.persistency, parameters.frequency_gain);

			// use the noise as height value
			terrain.position[idx].z = parameters.terrain_height*noise - parameters.terrain_height/2;

			// use also the noise as color value
			terrain.color[idx] = vec3(1,1,1);
		}
	}

	// Update the normal of the mesh structure
	terrain.compute_normal();
	
	// Update step: Allows to update a mesh_drawable without creating a new one
	terrain_visual.update_position(terrain.position);
	terrain_visual.update_normal(terrain.normal);
	terrain_visual.update_color(terrain.color);	
}

float evaluate_terrain_height_perlin(float u, float v, perlin_noise_parameters const& parameters)
{
			// Compute the Perlin noise
		float const noise = noise_perlin({u, v}, parameters.octave, parameters.persistency, parameters.frequency_gain);
			// use the noise as height value
		float z = parameters.terrain_height*noise;

			// use also the noise as color value
			//terrain.color[idx] = 0.3f*vec3(0,0.5f,0)+0.7f*noise*vec3(1,1,1);

    return z;
}


mesh montagne::create_terrain_mesh2(int N, float terrain_length,  perlin_noise_parameters const& parameters)
{

    mesh terrain; // temporary terrain storage (CPU only)
    terrain.position.resize(N*N);
    terrain.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            

            // Compute the real coordinates (x,y) of the terrain 
            float x = (u - 0.5f) * terrain_length;
            float y = (v - 0.5f) * terrain_length;

            // Compute the surface height function at the given sampled coordinate
            float z = evaluate_terrain_height_perlin(u,v, parameters);

            // Store vertex coordinates
            terrain.position[kv+N*ku] = {x,y,z};
            terrain.uv[kv+N*ku] = {ku*0.00050f,kv*0.00050f};
        }
    }

    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            unsigned int idx = kv + N*ku; // current vertex offset

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    // need to call this function to fill the other buffer with default values (normal, color, etc)
	terrain.fill_empty_field(); 

    return terrain;
}


void montagne::initialize(){
	int N = 1000;
	terrain_longueur = 800;
	hauteurGlace = 100;
	// Montagnes
	terrain_mesh = create_terrain_mesh2(N,terrain_longueur, parameters);
	terrain_drawable.initialize(terrain_mesh, "terrain");
	//update_terrain(terrain_mesh, terrain_drawable, parameters);
	//Lac 
	lac_mesh = create_terrain_mesh(terrain_longueur);
	lac_mesh.position+={0,0,hauteurGlace};
	lac_drawable.initialize(lac_mesh, "lac");
	
	// texture
	GLuint const texture_image_id = opengl_load_texture_image("assets/texture_montagne_10.jpg",
			GL_REPEAT,
			GL_REPEAT);
	
	terrain_drawable.texture = texture_image_id;

	GLuint const texture_lac_id = opengl_load_texture_image("assets/lac.jpeg",
		GL_CLAMP_TO_BORDER,
		GL_CLAMP_TO_BORDER);
	
	lac_drawable.texture = texture_lac_id;

	// positions des points de controle de la trajectoire des rails
	sphere2.initialize(mesh_primitive_sphere(1.0f), "pt_controle");
	sphere2.shading.color = { 1, 0, 0 };

}

std::vector<cgp::vec3> generate_positions_on_lac(int N, montagne terrain){
    std::vector<cgp::vec3> p_i;
    for(int i=0;i<N;++i){
        float x = rand_interval(-terrain.terrain_longueur, terrain.terrain_longueur);
        float y = rand_interval(-terrain.terrain_longueur, terrain.terrain_longueur);
        float z = evaluate_terrain_height_perlin(x/terrain.terrain_longueur/2+0.5f,y/terrain.terrain_longueur/2+0.5f, terrain.parameters);
		while (z>=terrain.hauteurGlace){
 			x = rand_interval(-terrain.terrain_longueur, terrain.terrain_longueur);
     	    y = rand_interval(-terrain.terrain_longueur, terrain.terrain_longueur);
     	    z = evaluate_terrain_height_perlin(x/terrain.terrain_longueur/2+0.5f,y/terrain.terrain_longueur/2+0.5f, terrain.parameters);
		}
		p_i.push_back(vec3({x, y, terrain.hauteurGlace}));

    }
    return p_i;
}
