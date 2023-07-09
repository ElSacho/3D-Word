#include "interpolation.hpp"


using namespace cgp;

void chemin::initialize(cgp::buffer<cgp::vec3> const& key_positions_arg, cgp::buffer<float> const& key_times_arg)
{
	key_positions = key_positions_arg;
	key_times = key_times_arg;
}


mesh chemin::createRail3(float largeur, float epaisseur, cgp::mesh& terrain, float tMin, float tMax)
{
    float r = 0.050f;
    mesh cube;
    mesh haut;
    mesh cote;
    mesh fullRail;

    float ecart = 0.1f;
    float t = tMin+ecart;
    float k = (tMax-tMin)/ecart-2;
    vec3 larg = {epaisseur,0,0};
    vec3 hauteur = {0,0,epaisseur};
    vec3 p1 = interpolation(t, key_positions, key_times, terrain);
    for(int i=0 ;i<k;++i){
        vec3 p2 = interpolation(t+ecart, key_positions, key_times, terrain);
        haut.push_back(mesh_primitive_quadrangle(p1, p1+larg, p2+larg, p2));
        cote.push_back(mesh_primitive_quadrangle(p1, p1+hauteur, p2+hauteur, p2));
        p1=p2;
        t=t+ecart;
    }
    haut.position+={largeur/2,0,0};
    cube.push_back(haut);
    haut.position+={0,0,epaisseur};
    cube.push_back(haut);

    cote.position+={largeur/2+epaisseur/2,0,0};
    cube.push_back(cote);
    cote.position+={-epaisseur,0,0};
    cube.push_back(cote);
    fullRail.push_back(cube);
    cube.position+={-largeur,0,0};
    fullRail.push_back(cube);

    return fullRail;
}


mesh chemin::createStructure(float largeur, float epaisseur, cgp::mesh& terrain, float tMin, float tMax, float hauteur){
    float ecart = 0.4f;
    float t = tMin;
    float k = (tMax-tMin)/ecart-2;
    mesh mesh_bois;
    mesh structure;
    for(int i=0 ;i<k;++i){
        t=t+ecart;
        vec3 pos = interpolation(t, key_positions, key_times, terrain);
        mesh bois = cylinder_with_texture(epaisseur, hauteur+ecart);
        bois.position+={pos[0],pos[1],pos[2]-(hauteur+ecart)/2};
        mesh_bois.push_back(bois);
    }
    mesh_bois.position+={-largeur/2,0,0};
    structure.push_back(mesh_bois);
    mesh_bois.position+={largeur,0,0};
    structure.push_back(mesh_bois);
    return structure;

}



mesh chemin::createBois(float largeur, float epaisseur, cgp::mesh& terrain, float tMin, float tMax){

    float ecart = 0.15f;
    float t = tMin+ecart;
    float k = (tMax-tMin)/ecart-2;

    vec3 p = interpolation(t, key_positions, key_times, terrain);
    mesh mesh_bois = create_mesh_rail(largeur, epaisseur,p);
    for(int i=0 ;i<k;++i){
        t=t+ecart;
        vec3 pos = interpolation(t, key_positions, key_times, terrain);
        mesh bois = create_bois(largeur, epaisseur,pos);
        mesh_bois.push_back(bois);
    }
    return mesh_bois;
}

void chemin::createAllRail(float largeur, float epaisseur, cgp::mesh& terrain,cgp::buffer<cgp::vec3> const& key_positions_arg, cgp::buffer<float> const& key_times_arg, float tMin, float tMax, float hauteur){
    key_positions = key_positions_arg;
	key_times = key_times_arg;

    mesh bois = this->createBois( largeur,  epaisseur, terrain, tMin, tMax);
    bois.position+={0,0,hauteur};
    boisRail.initialize(bois,"bois rail");

    mesh mesh_rail = this->createRail3( largeur,  epaisseur, terrain, tMin, tMax);
    mesh_rail.position+={0,0,hauteur};
    acierRail.initialize(mesh_rail,"acier rail");

    mesh mesh_struc = this->createStructure( largeur,  epaisseur, terrain, tMin, tMax, hauteur);
    mesh_struc.position+={0,0,hauteur};
    structurePortante.initialize(mesh_struc,"structure");
	
    acierRail.texture = opengl_load_texture_image("assets/acierRail.png");
    boisRail.texture = opengl_load_texture_image("assets/boisRail.jpeg");
    structurePortante.texture = opengl_load_texture_image("assets/trunk.jpg");


    hierarchy_rail.add(acierRail);
	//hierarchie piston 1
	hierarchy_rail.add(boisRail, "acier rail");
    hierarchy_rail.add(structurePortante, "acier rail");



    //

}


/** Compute the linear interpolation p(t) between p1 at time t1 and p2 at time t2*/
vec3 linear_interpolation(float t, float t1, float t2, vec3 const& p1, vec3 const& p2);

/** Compute the cardinal spline interpolation p(t) with the polygon [p0,p1,p2,p3] at time [t0,t1,t2,t3]
*  - Assume t \in [t1,t2] */
vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K, cgp::mesh& terrain);
vec3 rot_cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K, cgp::mesh& terrain);
vec3 cardinal_spline_interpolationSansGlace(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K);

/** Find the index k such that intervals[k] < t < intervals[k+1] 
* - Assume intervals is a sorted array of N time values
* - Assume t \in [ intervals[0], intervals[N-1] [       */
int find_index_of_interval(float t, buffer<float> const& intervals);


vec3 interpolation(float t, buffer<vec3> const& key_positions, buffer<float> const& key_times, cgp::mesh& terrain)
{
    // Find idx such that key_times[idx] < t < key_times[idx+1]
    int idx = find_index_of_interval(t, key_times);

    // Get parameters (time and position) used to compute the linear interpolation
    //   (You will need to get more parameters for the spline interpolation)
    float t0 = key_times[idx-1]; // = t_{i-1}
    float t1 = key_times[idx  ]; // = t_i
    float t2 = key_times[idx+1]; // = t_{i+1}
    float t3 = key_times[idx+2]; // = t_{i+2}

    vec3 const& p0 = key_positions[idx-1]; // = p_{i-1}
    vec3 const& p1 = key_positions[idx  ]; // = p_i
    vec3 const& p2 = key_positions[idx+1]; // = p_{i+1}
    vec3 const& p3 = key_positions[idx+2]; // = p_{i+1}
	
    // Call the interpolation
	// vec3 p = linear_interpolation(t, t1,t2, p1,p2);
    float K = 0.5f;
    vec3 p = cardinal_spline_interpolation(t, t0, t1, t2, t3, p0, p1, p2, p3, K, terrain);
    return p;
}


vec3 interpolationSansGlace(float t, buffer<vec3> const& key_positions, buffer<float> const& key_times)
{
    // Find idx such that key_times[idx] < t < key_times[idx+1]
    int idx = find_index_of_interval(t, key_times);

    // Get parameters (time and position) used to compute the linear interpolation
    //   (You will need to get more parameters for the spline interpolation)
    float t0 = key_times[idx-1]; // = t_{i-1}
    float t1 = key_times[idx  ]; // = t_i
    float t2 = key_times[idx+1]; // = t_{i+1}
    float t3 = key_times[idx+2]; // = t_{i+2}

    vec3 const& p0 = key_positions[idx-1]; // = p_{i-1}
    vec3 const& p1 = key_positions[idx  ]; // = p_i
    vec3 const& p2 = key_positions[idx+1]; // = p_{i+1}
    vec3 const& p3 = key_positions[idx+2]; // = p_{i+1}
	
    // Call the interpolation
	// vec3 p = linear_interpolation(t, t1,t2, p1,p2);
    float K = 0.5f;
    vec3 p = cardinal_spline_interpolationSansGlace(t, t0, t1, t2, t3, p0, p1, p2, p3, K);
    return p;
}

vec3 rotationTrain(float t, buffer<vec3> const& key_positions, buffer<float> const& key_times,cgp::mesh& terrain)
{
    // Find idx such that key_times[idx] < t < key_times[idx+1]
    int idx = find_index_of_interval(t, key_times);

    // Get parameters (time and position) used to compute the linear interpolation
    //   (You will need to get more parameters for the spline interpolation)
    float t0 = key_times[idx-1]; // = t_{i-1}
    float t1 = key_times[idx  ]; // = t_i
    float t2 = key_times[idx+1]; // = t_{i+1}
    float t3 = key_times[idx+2]; // = t_{i+2}

    vec3 const& p0 = key_positions[idx-1]; // = p_{i-1}
    vec3 const& p1 = key_positions[idx  ]; // = p_i
    vec3 const& p2 = key_positions[idx+1]; // = p_{i+1}
    vec3 const& p3 = key_positions[idx+2]; // = p_{i+1}
	
    // Call the interpolation
	// vec3 p = linear_interpolation(t, t1,t2, p1,p2);
    float K = 0.5f;
    vec3 rot = rot_cardinal_spline_interpolation(t, t0, t1, t2, t3, p0, p1, p2, p3, K,terrain);
    return rot;
}


/** Compute linear interpolation
 * 
      t: current time
      k: "current index" such that t_k < t < t_{k+1}, wxith t_k=key_time[k]
      t1: t_k 
      t2: t_{k+1} 
      (p1,p2): positions associated to time (t1,t2) */
vec3 linear_interpolation(float t, float t1, float t2, vec3 const& p1, vec3 const& p2)
{
    float const alpha = (t-t1)/(t2-t1);
    vec3 const p = (1-alpha)*p1 + alpha*p2;

    return p;
}

/** Compute the cardinal spline interpolation
      t: current time
      k: "current index" such that t_k < t < t_{k+1}, wxith t_k=key_time[k]
      t0: t_{k-1}
      t1: t_k
      t2: t_{k+1} 
      t3: t_{k+2}  
      (p0,p1,p2,p3): positions associated to time (t0,t1,t2,t3) 
      K: spline tension       */
vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K, cgp::mesh& terrain)
{
    // To do: fill the function to compute p(t) as a cardinal spline interpolation
    //vec3 const p = {0,0,0};
    float hauteurGlace =100;
    float s  = (t - t1) / (t2 - t1);
    vec3 d1 = 2*K*(p2 - p0) / (t2 - t0);
    vec3 d2 = 2*K*(p3 - p1) / (t3 - t1);

    vec3 p = (2*s*s*s - 3*s*s + 1)*p1 + (s*s*s - 2*s*s + s)*d1 + (-2*s*s*s + 3*s*s)*p2 + (s*s*s - s*s)*d2;
    if (p[2]<hauteurGlace){
        p[2]=hauteurGlace;
    }
    //vec3 h = {p[0],p[1],evaluate_terrain_height(p[0],p[1],terrain)};
    return p;
    // idee : t ne represente plus le temps total mais la proportion de segment realisee t=0.5 indique qu'on est a la moitie par exemple

}

vec3 cardinal_spline_interpolationSansGlace(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K)
{
    // To do: fill the function to compute p(t) as a cardinal spline interpolation
    //vec3 const p = {0,0,0};
    float s  = (t - t1) / (t2 - t1);
    vec3 d1 = 2*K*(p2 - p0) / (t2 - t0);
    vec3 d2 = 2*K*(p3 - p1) / (t3 - t1);

    vec3 p = (2*s*s*s - 3*s*s + 1)*p1 + (s*s*s - 2*s*s + s)*d1 + (-2*s*s*s + 3*s*s)*p2 + (s*s*s - s*s)*d2;

    //vec3 h = {p[0],p[1],evaluate_terrain_height(p[0],p[1],terrain)};
    return p;
    // idee : t ne represente plus le temps total mais la proportion de segment realisee t=0.5 indique qu'on est a la moitie par exemple
}


vec3 rot_cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, vec3 const& p0, vec3 const& p1, vec3 const& p2, vec3 const& p3, float K, cgp::mesh& terrain)
{
    // To do: fill the function to compute p(t) as a cardinal spline interpolation
    //vec3 const p = {0,0,0};

    float s  = (t - t1) / (t2 - t1);
    vec3 d1 = 2*K*(p2 - p0) / (t2 - t0);
    vec3 d2 = 2*K*(p3 - p1) / (t3 - t1);

    vec3 derivee1 = (6*s*s - 6*s)*p1 + (3*s*s - 4*s +1)*d1 + (-6*s*s+6*s)*p2 + (3*s*s-2*s)*d2;
    vec3 p = (2*s*s*s - 3*s*s + 1)*p1 + (s*s*s - 2*s*s + s)*d1 + (-2*s*s*s + 3*s*s)*p2 + (s*s*s - s*s)*d2;
    float s2  = (t2 - t1) / (t2 - t1);

    vec3 pente = derivee1*(s2-s);

    float degreZ = std::atan(pente[2]/pente[0]);
    float degreX = std::atan(pente[1]/pente[0]);



    vec3 deg = {0,degreZ,degreX};
    return deg;
}

int find_index_of_interval(float t, buffer<float> const& intervals)
{
    int const N = intervals.size();
    bool error = false;

    if (intervals.size() < 2) {
        std::cout<<"Error: Intervals should have at least two values; current size="<<intervals.size()<<std::endl;
        error = true;
    }
    if (N>0 && t < intervals[0]) {
        std::cout<<"Error: current time t is smaller than the first time of the interval"<<std::endl;
        error = true;
    }
    if(N>0 && t > intervals[N-1]) {
        std::cout<<"Error: current time t is greater than the last time of the interval"<<std::endl;
        error = true;
    }
    if (error == true) {
        std::string const error_str = "Error trying to find interval for t="+str(t)+" within values: ["+str(intervals)+"]";
        error_cgp( error_str );
    }


    int k=0;
    while( intervals[k+1]<t )
        ++k;
    return k;
}



float findConversion(cgp::buffer<vec2> conversion, float vraieDist){
    int N = conversion.size();
    if(conversion[0][1] > vraieDist){
        return 0;
    }
    for(int i=1;i<N;++i){
        if(conversion[i][1] > vraieDist){
            float t = conversion[i][0];
            return t;
        }
    }
    return 0;
}

float findConversionToVraieDist(cgp::buffer<cgp::vec2> conversion, float d){
    int N = conversion.size();
    if(conversion[0][0] > d){
        return 0;
    }
    for(int i=1;i<N;++i){
        if(conversion[i][0] > d){
            float t = conversion[i][1];
            return t;
        }
    }
    return 0;
}

mesh chemin::createRail(float largeur, float epaisseur, cgp::mesh& terrain)
{
    // on cree les rails sur les cotes
    float t = 1.5f;
    vec3 p = interpolation(t, key_positions, key_times, terrain);
    mesh mesh_rail = create_mesh_rail(largeur, epaisseur,p);
    
    for(int i=0 ;i<100;++i){
        t=t+0.10;
        vec3 pos = interpolation(t, key_positions, key_times, terrain);
        mesh mesh_railTemp = create_mesh_rail(largeur, epaisseur,pos);
        mesh_rail.push_back(mesh_railTemp);
    }

    return mesh_rail;
}




mesh chemin::createRail2(float largeur, float epaisseur, cgp::mesh& terrain)
{
    float r = 0.050f;

    // Number of samples of the terrain is N x N
    int N = 50;

    mesh cylinder; // temporary terrain storage (CPU only)
    cylinder.position.resize(N*N);
    cylinder.uv.resize(N*N);
    float t = 1.5f;
    // Fill terrain geometry
    for(int ku=0; ku<N; ++ku)
    {
        t=t+0.1;
        vec3 pos = interpolation(t, key_positions, key_times, terrain);
        for(int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            float u = ku/(N-1.0f);
            float v = kv/(N-1.0f);

            // Compute the local surface function
            vec3 p = {r*std::cos(2* Pi *u)+pos[0], 1*(v-0.5f)+pos[1], r*std::sin(2* Pi *u)+pos[2]};
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



mesh chemin::cylinder_with_texture2()
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
            vec3 p = { r*std::cos(2* Pi *u),h*(v-0.5f),  r*std::sin(2* Pi *u)};
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

cgp::buffer<cgp::vec2> convertirEnVraieDist(cgp::buffer<cgp::vec3> const& key_positions, cgp::buffer<float> const& key_times, cgp::mesh& terrain, float tMin, float tMax){
    cgp::buffer<vec2> conversion;
    float t = tMin+0.1f;
    float precision = 0.0001f;
    float vraieDist = 0;
    vec3 pos1 = interpolation(t, key_positions, key_times, terrain);
    vec3 pos2;
    vec3 pente;
    conversion.push_back(vec2{t,vraieDist});
    while(t<tMax-0.1f){
        t=t+precision;
        if (t>tMax){
            break;
        }
        pos2 = interpolation(t, key_positions, key_times, terrain);
        pente = pos2 - pos1;
	    vraieDist=vraieDist+std::sqrt((pente[0])*(pente[0])+(pente[1])*(pente[1])+(pente[2])*(pente[2]));
        pos1=pos2;
        conversion.push_back(vec2{t,vraieDist});
    }
    return conversion;
}