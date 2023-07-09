#pragma once

#include "cgp/cgp.hpp"


cgp::mesh disc_with_texture(float r);
cgp::mesh cylinder_with_texture(float r, float h);
cgp::mesh disc_with_hole(float r1, float r2);
cgp::mesh cylinder_with_hole(float r1, float r2, float h);
cgp::mesh rectangle3D(float h, float lon, float lar);
cgp::mesh biele_2tous(float r1, float r2,float h, float lon, float lar);
cgp::mesh biele_3tous(float r1, float r2,float h, float lon, float lar);
cgp::mesh mesh_wheel(float r1, float h);
cgp::mesh cylinder_with_ecart(float r, float h, float e);
cgp::mesh vis_with_texture(float r, float h);
cgp::mesh disc_broker(float r1);
cgp::mesh create_mesh_rail(float largeur, float epaisseur, cgp::vec3 position);
cgp::mesh create_bois(float largeur, float epaisseur, cgp::vec3 position);
cgp::mesh trouPourPoisson(float r1, float r2, float h);


cgp::mesh torus_with_texture();
cgp::mesh wheel_with_texture();
cgp::mesh biele1();



