#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_DEFAULT_3D_GRAPHICS


struct scene_model : scene_base
{

    /** A part must define two functions that are called from the main function:
     * setup_data: called once to setup data before starting the animation loop
     * frame_draw: called at every displayed frame within the animation loop
     *
     * These two functions receive the following parameters
     * - shaders: A set of shaders.
     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
     * - data: The part-specific data structure defined previously
     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
    */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


    // Function used to set elements of the graphical interface
    void set_gui();
    bool is_wireframe = false;

    /** Part-specific data
    This structure contains the data that are specific to the given part.
    Every part may use different data. */

    // Visual representation of a plane
    vcl::mesh_drawable plane;

    // Visual representation of a cube
    vcl::mesh_drawable cube;

    // Visual representation of a cylinder
    vcl::mesh_drawable cylinder;

    // Visual representation of a sphere
    vcl::mesh_drawable sphere;

    // Visual representation of a curve
    vcl::curve_drawable curve;

    // Visual representation of a set of segments
    vcl::curve_drawable polygon_segments;

    // Timer used for the animation
    vcl::timer_basic timer;

    std::vector<vcl::vec3> positions;
    vcl::mesh_drawable cone;
    vcl::mesh_drawable cylinder_tree;

};

#endif


