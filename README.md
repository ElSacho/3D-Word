# Orient Express Project

## Compilation Instructions

The project is written in C++ and uses the CGP library. Detailed steps for compiling this project can be found on the CGP website's compilation guide. Please follow the guide to ensure the proper setup of your project.

Visit the link below for comprehensive instructions:

[CGP Compilation Guide](https://imagecomputing.net/cgp/compilation/content/01_compilation/index.html)

This guide provides step-by-step instructions on how to set up your environment and compile C++ projects using the CGP library. Be sure to follow all the steps outlined in the guide to successfully compile and run this project.

Should you encounter any issues during the compilation process, please refer back to the guide, as it covers common issues and their solutions.

Train animation is generative: the initial speed is given as input, and everything is updated as a function of accumulated speed and terrain gradient.

## Original Idea

Our project aims to present a scene featuring the legendary Orient Express, traveling through a snowy, mountainous landscape.

## Landscape

We began our scene modeling with the mountainous landscape. To achieve this, we surrounded our scene with a skybox displaying a mountainous backdrop. Due to the mediocre quality of freely available skyboxes on the internet, we decided to consider only the vertical planes (4 faces), replicating the same image for each. The drawback lies in "anomalies" at the junction of two faces, showing a perfectly symmetrical landscape.

For the terrain itself, we created a 400 by 400 grid, to which we then applied Perlin noise with empirically derived parameters to replicate the mountainous landscape as accurately as possible. We added a 2D mountain texture (rocks and snow) from an image found on the internet. Although the image quality was a primary criterion in our texture choice, it does not make the terrain appear realistic enough when zoomed in too closely.

We decided to enhance the scene by adding an ice lake and details like igloos, a fisherman, and flies. The lake is a plan at z = constant, on which we added a 2D ice texture, and that cuts through the Perlin noise of the mountains.

### Details

#### Igloos
For the igloos, we created a specific mesh (a deformed sphere), applied a suitable texture, and randomly distributed them on the lake (without detecting potential contacts or penetrations between two igloos). The shapes of the igloos are quite random, as the radius and deformation of the sphere, as well as the height and size of the door, are generated randomly.

#### Fisherman
For the fisherman, we first created a hole in the lake, modeled by a hollow disc where the center of the disc is slightly raised, giving the illusion of a hole. The fisherman is located at the edge of the hole, with his arm animated descriptively. The fishing line held in his hand is animated by a generator movement: we chose an extendable thread model (mass-spring system), and we constrain the movement of the top point to match the fisherman's hand. The extendable nature of the line does not harm realism since the viewer does not see the lower end of the line.

#### Flies
The flies around the fisherman are animated by a boyds movement: a "leader" fly follows a circular trajectory around the fishing hole, and the other flies tend to approach or move away based on their distance from it.

#### Snow
For falling snow, we initially opted for flakes modeled by white spheres, to which we assigned a mass, subjected to gravitational forces and the viscous drag force of the air. This choice posed two major problems: first, it required the management of a large number of spheres simultaneously, which affected the visual rendering of the scene, making it jerky; second, realism was lost with the assimilation of flakes to simple white spheres, especially for those falling near the camera.

Therefore, we chose to represent the flakes with billboards, which solves both previously stated problems. We animated these billboards with uniform motion (constant speed) from a random position taken at a fixed altitude. Indeed, it seemed to us that the acceleration of the flakes during their fall was not an important detail for the realism of the scene. Below a certain altitude (once the snowflake has passed the zero level of the scene), we remove it from the list of "active" flakes to prevent the algorithm from diverging, having to manage too many flakes.

## Train

For the train, we modeled a locomotive and some wagons (we displayed 6 in our project, but more can easily be shown). To animate the train, we placed trajectory control points on our terrain. We then interpolated these points with a spline curve to describe the trajectory. Given the mountainous terrain, the trajectory is 3D in space and cannot be restricted to a plane, which created problems for train rotations.

The train animation is generative. We indicate an acceleration and an initial speed, and the train then moves on its own. The speed can increase or even decrease depending on the slope. Thus, the train's acceleration and speed depend on the terrain and the previous speed. The idea of the code is as follows:

Based on previous data and the terrain, we update the acceleration, then the speed, to calculate the distance covered. We calculate the position through interpolation at this distance and update the position of the locomotive. Thanks to the derivative of the interpolation curve, we can update the rotation along the y-axis. We had issues related to abrupt changes in the sign of this angle, so we coded a system preventing abrupt angle changes. This method works far less well for the rotation that allows the train to follow the curve. For that, we used the height difference between two successive positions.

The distance used is the actual theoretical distance covered by the train and not the "t" parameter of the initial interpolation code seen in class in a real covered distance. The interest is threefold:

Maintaining consistent speed: Even if two control points are very close or far apart, the train goes at the same speed on the sections. The train's speed does not depend on the gap between control points.
Coding the movement of the wagons. To keep consistency in the train movement, we required the wagons to always be spaced at the same real distance. Thus, the wagon movements are always at the same distance from the train.
Rotating the wheels with some consistency. The wheel rotation speed depends on the real speed.
The wheel rotation speed is particularly important in our project since we modeled a piston for the locomotive. All pieces of the piston wheel have been modeled in "hard" code and are available in the "model_texture" file. The piston drives the wheels, and the wheels turn according to the piston... at least theoretically, because in reality the piston movement is descriptive. We have equated the piston positions based on the wheel rotation.

Finally, for the train to move, it must have rails! To do this, we created a mesh, also with "hard" OpenGL code. We provide the train's trajectory and go through it step by step. At each step, we add four points to form a square and, at the end of the process, obtain a giant, very thin rectangle. We duplicate the mesh and finally add a steel texture on top. Then we proceed similarly to gradually add wooden boards between the rails and stilts to support the structure.

## Conclusion

In conclusion, this project was an interesting journey through the challenges of 3D animation. Through our attempts to render a realistic Orient Express traveling through a snowy, mountainous landscape, we were able to delve deeper into the complexities of 3D modeling, texturing, and animation. Although certain aspects posed difficulties, such as train rotations and the rendering of falling snow, the solutions we found, like using billboards for the snowflakes and a generative animation for the train, have provided valuable insights into the world of computer graphics. The project has been a valuable learning experience, offering us the chance to put theory into practice.

