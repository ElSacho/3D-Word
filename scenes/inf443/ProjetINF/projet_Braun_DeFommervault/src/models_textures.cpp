
#include "models_textures.hpp"

using namespace cgp;

mesh torus_with_texture()
{
    float a = 1.7f;
    float b = 0.6f;

    // Number of samples of the terrain is N x N
    int N = 50;

    mesh torus; // temporary terrain storage (CPU only)
    torus.position.resize(N*N);
    torus.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function

            vec3 p = {
					(a + b*std::cos(2* Pi *u))*std::cos(2* Pi *v),
					(a + b*std::cos(2* Pi *u))*std::sin(2* Pi *v),
					b*std::sin(2* Pi *u)};
            

            // Store vertex coordinates
            torus.position[kv+N*ku] = p;
            torus.uv[kv+N*ku] = {v*(N-1.0f)/3.265,u*(N-1.0f)/6};
        }
    }


    // Generate triangle organization
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            unsigned int idx = kv + N*ku;

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            torus.connectivity.push_back(triangle_1);
            torus.connectivity.push_back(triangle_2);
        }
    }

    torus.fill_empty_field();
    return torus;
}


mesh cylinder_with_texture()
{
    float r = 1.0f;
    float h = 4.0f;

    // Number of samples of the terrain is N x N
    int N = 50;

    mesh cylinder; // temporary terrain storage (CPU only)
    cylinder.position.resize(N*N);
    cylinder.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function
            vec3 p = {r*std::cos(2* Pi *u), r*std::sin(2* Pi *u), h*(v-0.5f)};
            vec2 uv = {u,v};

            // Store vertex coordinates
            cylinder.position[kv+N*ku] = p;
            cylinder.uv[kv+N*ku] = {u,v};
        }
    }

    // Generate triangle organization
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            int idx = kv + N*ku;

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            cylinder.connectivity.push_back(triangle_1);
            cylinder.connectivity.push_back(triangle_2);
        }
    }

	cylinder.fill_empty_field();
    return cylinder;
}


mesh disc_with_texture(float r)
{

	mesh disc;
    int N = 50;

	for (int k = 0; k < N; ++k)
	{
		float u = k/(N-1.0f);
		vec3 p = r * vec3(std::cos(2* Pi *u), std::sin(2* Pi *u), 0.0f);
		disc.position.push_back(p);
        disc.uv.push_back({0.5f + std::cos(2* Pi *u)/2, 0.5f + std::sin(2* Pi *u)/2});
		
	}
	// middle point
    disc.position.push_back({0,0,0});
    disc.uv.push_back({0.5f ,0.5f});
    

	for (int k = 0; k < N-1; ++k)
		disc.connectivity.push_back( uint3{ N, k, k+1});

	disc.fill_empty_field();
    return disc;
}



mesh wheel_with_texture()
{
    float r = 1.0f;
    float h = 0.01f;

    // Number of samples of the terrain is N x N
    int N = 20;

    mesh cylinder; // temporary terrain storage (CPU only)
    cylinder.position.resize(N*N);
    cylinder.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function
            vec3 p = {r*std::cos(2* Pi *u), r*std::sin(2* Pi *u), h*(v-0.5f)};
            vec2 uv = {u,v};

            // Store vertex coordinates
            cylinder.position[kv+N*ku] = p;
            cylinder.uv[kv+N*ku] = {u,v};
        }
    }

    // Generate triangle organization
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            int idx = kv + N*ku;

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            cylinder.connectivity.push_back(triangle_1);
            cylinder.connectivity.push_back(triangle_2);
        }
    }

	cylinder.fill_empty_field();
    return cylinder;
}

mesh vis_with_texture(float r, float h)
{
    mesh cylinder = cylinder_with_texture(r, h);
    mesh rond = disc_with_texture(r);
    rond.position += {0,0,h/2};
    cylinder.push_back(rond);
    rond.position += {0,0,-h};
    cylinder.push_back(rond);
    return cylinder;
}

mesh cylinder_with_texture(float r, float h)
{

    // Number of samples of the terrain is N x N
    int N = 50;

    mesh cylinder; // temporary terrain storage (CPU only)
    cylinder.position.resize(N*N);
    cylinder.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function
            vec3 p = {r*std::cos(2* Pi *u), r*std::sin(2* Pi *u), h*(v-0.5f)};
            vec2 uv = {u,v};

            // Store vertex coordinates
            cylinder.position[kv+N*ku] = p;
            cylinder.uv[kv+N*ku] = {u,v};
        }
    }

    // Generate triangle organization
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            int idx = kv + N*ku;

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            cylinder.connectivity.push_back(triangle_1);
            cylinder.connectivity.push_back(triangle_2);
        }
    }

	cylinder.fill_empty_field();
    return cylinder;
}


mesh disc_with_hole(float r1, float r2)
{
    //r1>r2
	mesh disc;
    int N = 50;

	for (int k = 0; k < N; ++k)
	{
		float u = k/(N-1.0f);
		vec3 p = r1 * vec3(std::cos(2* Pi *u), std::sin(2* Pi *u), 0.0f);
		disc.position.push_back(p);
        //disc.uv.push_back({0.5f + std::cos(2* Pi *u)/2, 0.5f + std::sin(2* Pi *u)/2});
		vec3 p2 = r2 * vec3(std::cos(2* Pi *u), std::sin(2* Pi *u), 0.0f);
        disc.position.push_back(p2);

		
	}
	// middle point
    //disc.uv.push_back({0.5f ,0.5f});
    

	for (int k = 0; k < N-1; ++k){
		disc.connectivity.push_back( uint3{ 2*k, 2*k+1, 2*k+2});
        disc.connectivity.push_back( uint3{ 2*k+2, 2*k+1, 2*k+3});
    }

	disc.connectivity.push_back( uint3{ 2*N-2, 2*N-1, 1});
    disc.connectivity.push_back( uint3{ 1, 2*N-1, 2});

	disc.fill_empty_field();
    return disc;
}

mesh cylinder_with_hole(float r1, float r2, float h)
{
    mesh disc = disc_with_hole(r1, r2);
    disc.position += {0,0,h};
    disc.push_back(disc_with_hole(r1, r2));
    disc.position += {0,0,-h};


    mesh cylindre = cylinder_with_texture(r1, h);
    cylindre.push_back(cylinder_with_texture(r2, h));

    disc.position += {0,0,h/2};
    cylindre.push_back(disc);

    return cylindre;
	

}

mesh rectangle3D(float h, float lon, float lar){
    mesh cube = mesh_primitive_quadrangle(vec3{-lon/2, -h/2, -lar/2}, vec3{lon/2, -h/2, -lar/2}, vec3{lon/2, h/2, -lar/2}, vec3{-lon/2, h/2, -lar/2});
    cube.push_back(mesh_primitive_quadrangle(vec3{-lon/2, h/2, -lar/2}, vec3{lon/2, h/2, -lar/2}, vec3{lon/2, h/2, lar/2}, vec3{-lon/2, h/2, lar/2}));
    cube.push_back(mesh_primitive_quadrangle(vec3{-lon/2, -h/2, lar/2}, vec3{lon/2, -h/2, lar/2}, vec3{lon/2, h/2, lar/2}, vec3{-lon/2, h/2, lar/2}));
    cube.push_back(mesh_primitive_quadrangle(vec3{-lon/2, -h/2, -lar/2}, vec3{lon/2, -h/2, -lar/2}, vec3{lon/2, -h/2, lar/2}, vec3{-lon/2, -h/2, lar/2}));
    cube.push_back(mesh_primitive_quadrangle(vec3{-lon/2, -h/2, -lar/2}, vec3{-lon/2, h/2, -lar/2}, vec3{-lon/2, h/2, lar/2}, vec3{-lon/2, -h/2, lar/2}));
    cube.push_back(mesh_primitive_quadrangle(vec3{lon/2, -h/2, -lar/2}, vec3{lon/2, h/2, -lar/2}, vec3{lon/2, h/2, lar/2}, vec3{lon/2, -h/2, lar/2}));

    return cube;

}



mesh biele_2tous(float r1, float r2,float h, float lon, float lar){
    mesh rond1 = cylinder_with_hole(r1, r2, h);
    rond1.position += {lon/2+r2/2,0,0};
    mesh rond2 = cylinder_with_hole(r1, r2, h);
    rond2.position += {-lon/2-r2/2,0,0};
    mesh rec1 = rectangle3D(h,lon,lar);
    rec1.push_back(rond1);
    rec1.push_back(rond2);
    rec1.position += {lon/2+r2/2,0,0};
    return rec1;
}

mesh biele_3tous(float r1, float r2,float h, float lon, float lar){
    mesh rond1 = cylinder_with_hole(r1, r2, h);
    rond1.position += {lon/2+r2/2,0,0};
    mesh rond2 = cylinder_with_hole(r1, r2, h);
    rond2.position += {-lon/2-r2/2,0,0};
    mesh rec1 = rectangle3D(h,lon,lar);
    mesh rec2 = rectangle3D(h,lon,lar);
    mesh rond3 = cylinder_with_hole(r1, r2, h);
    rond3.position += {lon/2+r2/2,0,0};
    rec2.push_back(rond3);
    rec1.push_back(rond1);
    rec1.push_back(rond2);
    rec2.position += {lon+r2,0,0};
    rec1.push_back(rec2);
    rec1.position += {lon/2+r2/2,0,0};
    return rec1;
}

mesh disc_broker(float r1)
{
    //r1>r2
	mesh disc;
    int N = 20;
	float u1 = 0/(N-1.0f)*2*Pi/4+3*Pi/4 ;
	vec3 p1 = r1 * vec3(std::cos(u1), std::sin(u1), 0.0f);
    float r2 = r1 * std::cos(u1);
	disc.position.push_back(p1);
	for (int k = 1; k < N-1; ++k)
	{
		float u = k/(N-1.0f)*2*Pi/4+3*Pi/4 ;
		vec3 p = r1 * vec3(std::cos(u), std::sin(u), 0.0f);
		disc.position.push_back(p);
        //disc.uv.push_back({0.5f + std::cos(2* Pi *u)/2, 0.5f + std::sin(2* Pi *u)/2});
		vec3 p2 = vec3(r2, r1*std::sin(u), 0.0f);
        disc.position.push_back(p2);	
	}
    	float u2 = (N-1)/(N-1.0f)*2*Pi/4+3*Pi/4 ;
		vec3 p3 = r1 * vec3(std::cos(u2), std::sin(u2), 0.0f);
		disc.position.push_back(p3);
	// middle point
    //disc.uv.push_back({0.5f ,0.5f});
    
	disc.connectivity.push_back( uint3{ 0, 1, 2});
	//for (int k = 0; k < 1; ++k){
    for (int k = 0; k < N-3; ++k){
		disc.connectivity.push_back( uint3{ 2*k+1, 2*k+3, 2*k+2});
        disc.connectivity.push_back( uint3{ 2*k+2, 2*k+3, 2*k+4});
    }

	disc.connectivity.push_back( uint3{ 2*N-5, 2*N-3, 2*N-4});
	disc.fill_empty_field();
    return disc;
}


mesh mesh_wheel(float r1, float h){

	mesh pneu = cylinder_with_texture(r1,h);
    mesh cylindre1 = cylinder_with_hole(r1*0.2/0.5, r1*0.1/0.5, h);
    pneu.push_back(cylindre1);
    std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    mesh cylindre2 = cylinder_with_hole(r1, r1*0.46/0.5, h);
    pneu.push_back(cylindre2);

    int N = 15;
    mesh cyclindreBiele = cgp::mesh_primitive_cylinder(r1*0.05/0.5, {r1*0.1/0.5*std::cos(2*0*Pi/N),r1*0.1/0.5*std::sin(2*0*Pi/N),0}, {r1*std::cos(2*0*Pi/N),r1*std::sin(2*0*Pi/N),0}, 20, 20, false);
    pneu.push_back(cyclindreBiele);
    for (int k = 1; k < N; ++k) 
    {
        mesh cyclindreBiele = cgp::mesh_primitive_cylinder(r1*0.03/0.5, {r1*0.1/0.5*std::cos(2*k*Pi/N),r1*0.1/0.5*std::sin(2*k*Pi/N),0}, {r1*std::cos(2*k*Pi/N),r1*std::sin(2*k*Pi/N),0}, 20, 20, false);
        pneu.push_back(cyclindreBiele);
    }
    mesh ro = disc_broker(r1);
    ro.position += { 0,0,h/2};
    pneu.push_back(ro);
    ro.position += { 0,0,-h};
    pneu.push_back(ro);

    mesh vis = vis_with_texture(r1*0.05/0.5,h);
    vis.position += { r1/2,0,h};
    pneu.push_back(vis);
    
	float u1 = 3*Pi/4 ;
	vec3 p1 = r1 * vec3(std::cos(u1), std::sin(u1), 0.0f);
    float u2 = 5*Pi/4 ;
	vec3 p2 = r1 * vec3(std::cos(u2), std::sin(u2), 0.0f);
    mesh cube = mesh_primitive_quadrangle(p1+vec3{0,0,h/2}, p2+vec3{0,0,h/2}, p2+vec3{0,0,-h/2}, p1+vec3{0,0,-h/2});
    pneu.push_back(cube);
    return pneu;

}

mesh cylinder_with_ecart(float r, float h, float e)
{

    // Number of samples of the terrain is N x N
    int N = 50;

    mesh cylinder; // temporary terrain storage (CPU only)
    cylinder.position.resize(N*N);
    cylinder.uv.resize(N*N);

    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function
            vec3 p = {r*std::cos(2* Pi *u), r*std::sin(2* Pi *u), h*(v-0.5f)};
            vec2 uv = {u,v};

            // Store vertex coordinates
            cylinder.position[kv+N*ku] = p;
            cylinder.uv[kv+N*ku] = {u,v};
        }
    }

    // Generate triangle organization
    for(int ku=0; ku<N-1; ++ku)
    {
        for(int kv=0; kv<N-1; ++kv)
        {
            int idx = kv + N*ku;

            uint3 triangle_1 = {idx, idx+1+N, idx+1};
            uint3 triangle_2 = {idx, idx+N, idx+1+N};

            cylinder.connectivity.push_back(triangle_1);
            cylinder.connectivity.push_back(triangle_2);
        }
    }

	cylinder.fill_empty_field();
    cylinder.position += { e,0,0};
    return cylinder;
}

mesh create_mesh_rail(float largeur, float epaisseur, cgp::vec3 position){
    mesh rail1 = rectangle3D(epaisseur,epaisseur,epaisseur);
    rail1.position += {-largeur/2,0,0};
    
    mesh rail2 = rectangle3D(epaisseur,epaisseur,epaisseur);
    rail2.position += {largeur/2,0,0};
    rail1.push_back(rail2);
    rail1.position += position;
    return rail1;
}

mesh create_bois(float largeur, float epaisseur, cgp::vec3 position){
    mesh rail1 = rectangle3D(largeur/4,largeur,epaisseur);
    rail1.position += position;
    return rail1;
}

mesh trouPourPoisson(float r1, float r2, float h)
{
    //r1>r2
	mesh disc;
    int N = 50;

	for (int k = 0; k < N; ++k)
	{
		float u = k/(N-1.0f);
		vec3 p = r1 * vec3(std::cos(2* Pi *u), std::sin(2* Pi *u), 0.0f);
		disc.position.push_back(p);
        disc.uv.push_back({0.5f + std::cos(2* Pi *u)/2, 0.5f + std::sin(2* Pi *u)/2});
		vec3 p2 = r2 * vec3(std::cos(2* Pi *u), std::sin(2* Pi *u), h);
        disc.position.push_back(p2);
        disc.uv.push_back({0.5f + std::cos(2* Pi *u)/2*r2/r1, 0.5f + std::sin(2* Pi *u)/2*r2/r1});



	}
	// middle point
    //disc.uv.push_back({0.5f ,0.5f});
    

	for (int k = 0; k < N-1; ++k){
		disc.connectivity.push_back( uint3{ 2*k, 2*k+1, 2*k+2});
        disc.connectivity.push_back( uint3{ 2*k+2, 2*k+1, 2*k+3});
    }

	disc.connectivity.push_back( uint3{ 2*N-2, 2*N-1, 1});
    disc.connectivity.push_back( uint3{ 1, 2*N-1, 2});

	disc.fill_empty_field();
    mesh cyl = cylinder_with_texture(r2,h);
    disc.push_back(cyl);
    return disc;
}


