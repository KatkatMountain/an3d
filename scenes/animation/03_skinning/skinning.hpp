#pragma once

#include "main/scene_base/base.hpp"
#ifdef SCENE_SKINNING

#include "quaternion.hpp"




// Connectivity information of a joint in the hierarchy
//  Store parent index, and the current name
struct joint_connectivity
{
    int parent;
    std::string name;
};

// 3D Geometry of a joint (position p, and rotation r)
struct joint_geometry
{
    vcl::vec3 p;
    quaternion r;
};

// Key pose of a joint (Key-time, and geometry at this time)
struct joint_geometry_time
{
    float time;
    joint_geometry geometry;
};

// Storage of the influence of a joint for a given vertex
struct skinning_influence
{
    int joint;    // index of the corresponding joint
    float weight; // skinning weight of this joint
};

// Structure storing skeleton data to perform skinning afterward
struct skeleton_structure
{
    vcl::buffer<joint_connectivity> connectivity;           // Connectivity of the skeleton
    vcl::buffer<joint_geometry>     rest_pose;              // Skeleton of the rest pose expressed in local coordinates
    vcl::buffer<vcl::buffer<joint_geometry_time> > anim;    // Skeleton animation expressed in local coordinates (N_joint x N_time)
};

// Storage structure to perform skinning deformation of a surface
struct skinning_structure
{
    vcl::buffer< vcl::buffer<skinning_influence> > influence; // Skinning weights: for each vertex, store all influence values (bone+weight)
    vcl::buffer<vcl::vec3> rest_pose;                         // 3D position of the mesh in rest pose
    vcl::buffer<vcl::vec3> rest_pose_normal;                  // 3D normals of the mesh in rest pose
    vcl::mesh deformed;                                       // Deformed mesh
};

enum gui_parameters_display_type {display_character, display_cylinder, display_bar};
struct gui_parameters
{
    bool display_skeleton_bones;
    bool display_skeleton_joints;
    bool display_skeleton_frames;
    bool display_mesh;
    bool display_rest_pose;
    bool display_wireframe;
    bool display_texture;
    int display_type;

};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui();


    skeleton_structure skeleton;
    skinning_structure skinning;
    vcl::mesh_drawable character_visual;

    vcl::segment_drawable_immediate_mode segment_drawer;
    vcl::mesh_drawable sphere;
    GLuint shader_mesh;

    vcl::mesh_drawable frame;

    gui_parameters gui_param;

    vcl::timer_interval timer;
};








#endif
