#include "flocon.hpp"

using namespace cgp;

void flocon::initialize(){

	// Flocons
	/*float const sphere_radius = 0.02f;
	sphere.initialize(mesh_primitive_sphere(sphere_radius), "sphere");
	sphere.shading.color = { 1, 1, 1 };*/

	
	// billboards flocons
	mesh quad_mesh = mesh_primitive_quadrangle({ -20, 0, 0 }, { 20, 0, 0 }, { 20, 0, 40 }, { -20, 0, 40 });
	quad_1.initialize(quad_mesh, "Quad 1");
	quad_1.texture = opengl_load_texture_image("assets/flocon_billboard_5.png");
	quad_1.shading.phong = { 0.4f, 0.6f,0,1 };
	quad_1.transform.translation = { 0,-0.5f,0 };

}

void flocon::update(){

	// Update the current time
	float dt = timer.update();

	// flocons
	bool const new_particle = timer.event;
	if (new_particle == true) {
		for (int i = 0; i<100 ; i++){
			vec3 const p0 = { rand_interval(-200, 200), rand_interval(-200, 200), 150};
			const vec3 v0 = vec3(0, 0, -rand_interval(5, 10));
			//const float s0 = rand_interval(0.02f, 0.05f);
			this->particles.push_back({ p0, v0});
		}
	}


	// Evolve position of billboards
	
	for (particle_structure& particle : particles){
		// evolution a vitesse constante = suffit au realisme (on ne voit pas les flocons "accelerer" en vrai)
		vec3& p = particle.p;
		vec3& v = particle.v;
		p = p + dt * v;
	}

	for (auto it = this->particles.begin(); it != this->particles.end(); ) {
		if (it->p.z < -0.5)
			it = this->particles.erase(it);
		if (it != this->particles.end())
			++it;
	}

}


//template <typename SCENE_ENVIRONMENT>
void flocon::display_semiTransparent(const cgp::scene_environment_basic_camera_spherical_coords environment2)
{
	for (particle_structure& particle : particles)
	{
            
        // Enable use of alpha component as color blending for transparent elements
        //  alpha = current_color.alpha
        //  new color = previous_color * alpha + current_color * (1-alpha)
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Disable depth buffer writing
        //  - Transparent elements cannot use depth buffer
        //  - They are supposed to be display from furest to nearest elements
        glDepthMask(false);


        // Re-orient the shape to always face the camera direction
        vec3 const front = normalize(environment2.camera.front() * vec3 { 1, 1, 0 }); // front-vector of the camera without z-component
        vec3 const right = environment2.camera.right();

        // Rotation such that R*{1,0,0} = right-direction, R*{0,1,0} = front-direction
        rotation_transform R = rotation_transform::between_vector({ 1,0,0 }, { 0,1,0 }, right, front);
        
        quad_1.transform.rotation = R;
        quad_1.transform.translation = particle.p;

        // Sort transparent shapes by depth to camera
        // This step can be skipped, but it will be associated to visual artifacts

        // Transform matrix (the same matrix which is applied in the vertices in the shader: T = Projection x View)
        mat4 T = environment2.projection.matrix() * environment2.camera.matrix_view();
        // Projected vertices (center of quads) in homogeneous coordinates
        vec4 p1 = T * vec4{ 0, -0.5f, 0, 1 };
        //vec4 p2 = T * vec4{ 0, +0.5f, 0, 1 };

        // Depth to camera
        float z1 = p1.z / p1.w;
        
		
		draw(quad_1, environment2);
		
		

        // Don't forget to re-activate the depth-buffer write
        glDepthMask(true);
        glDisable(GL_BLEND);
    }
}