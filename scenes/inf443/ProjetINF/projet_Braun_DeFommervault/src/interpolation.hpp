#pragma once


#include "cgp/cgp.hpp"
#include "terrain2.hpp"
#include "models_textures.hpp"


// Compute the interpolated position p(t) given a time t and the set of key_positions and key_frame
cgp::vec3 interpolationSansGlace(float t, cgp::buffer<cgp::vec3> const& key_positions, cgp::buffer<float> const& key_times);
cgp::vec3 interpolation(float t, cgp::buffer<cgp::vec3> const& key_positions, cgp::buffer<float> const& key_times, cgp::mesh& terrain);
cgp::vec3 rotationTrain(float t, cgp::buffer<cgp::vec3> const& key_positions, cgp::buffer<float> const& key_times, cgp::mesh& terrain);
cgp::buffer<cgp::vec2> convertirEnVraieDist(cgp::buffer<cgp::vec3> const& key_positions, cgp::buffer<float> const& key_times, cgp::mesh& terrain, float tMin, float tMax);
float findConversion(cgp::buffer<cgp::vec2> conversion, float vraieDist);
float findConversionToVraieDist(cgp::buffer<cgp::vec2> conversion, float d);


struct chemin{
    cgp::buffer<cgp::vec3> key_positions;
    cgp::buffer<cgp::vec3> key_positions2;
    cgp::buffer<cgp::vec3> key_positions3;
    cgp::buffer<float> key_times;
    cgp::mesh_drawable rail;
    cgp::mesh mesh_rail;
    cgp::mesh mesh_railTemp;

    cgp::mesh mesh_bois;
    cgp::mesh bois;

    cgp::mesh_drawable boisRail;
    cgp::mesh_drawable acierRail;
    cgp::mesh_drawable structurePortante;





    cgp::hierarchy_mesh_drawable hierarchy_rail;





    void initialize(cgp::buffer<cgp::vec3> const& key_positions_arg, cgp::buffer<float> const& key_times_arg);
    cgp::mesh createRail(float largeur, float epaisseur,cgp::mesh& terrain);
    cgp::mesh createRail2(float largeur, float epaisseur,cgp::mesh& terrain);
    cgp::mesh createRail3(float largeur, float epaisseur,cgp::mesh& terrain, float tMin, float tMax);
    cgp::mesh createBois(float largeur, float epaisseur, cgp::mesh& terrain, float tMin, float tMax);
    cgp::mesh cylinder_with_texture2();
    cgp::mesh createStructure(float largeur, float epaisseur, cgp::mesh& terrain, float tMin, float tMax, float hauteur);

    void createAllRail(float largeur, float epaisseur, cgp::mesh& terrain,cgp::buffer<cgp::vec3> const& key_positions_arg, cgp::buffer<float> const& key_times_arg, float tMin, float tMax,float hauteur);

};