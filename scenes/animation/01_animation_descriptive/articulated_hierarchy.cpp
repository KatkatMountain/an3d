
#include "articulated_hierarchy.hpp"


#ifdef SCENE_ARTICULATED_HIERARCHY


using namespace vcl;



void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    const float radius_body = 0.25f;
    const float radius_head = 0.15f;
    const float length_wing = 0.15f;
    
    //const float radius_arm = 0.05f;
    //const float length_arm = 0.2f;

    // The geometry of the body is a sphere
    mesh_drawable body = mesh_drawable(mesh_primitive_sphere(radius_body, {0,0,0}, 40, 40));
    body.uniform.transform.scaling_axis = {0.75,0.75,1.25};
    
    mesh_drawable head = mesh_drawable(mesh_primitive_sphere(radius_head, {0,0,0}, 40, 40));

    // Geometry of the eyes: black spheres
    mesh_drawable eye = mesh_drawable(mesh_primitive_sphere(0.03f, {0,0,0}, 20, 20));
    eye.uniform.color = {0,0,0};

    mesh_drawable wing = mesh_primitive_quad({-length_wing,0,-length_wing},
        {0.1,0,-length_wing},{0.1,0,length_wing},{-length_wing,0,length_wing});

    mesh_drawable wing2 = mesh_primitive_quad({0,0,-length_wing},
        {-length_wing,0,-length_wing},{-length_wing,0,0},{0,0,length_wing});
    // ------------------------------------
    // Shoulder part and arm are displayed as cylinder
    //mesh_drawable shoulder = mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    //mesh_drawable arm = mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm/1.5f,-length_arm/1.0f,0});

    // An elbow displayed as a sphere
    //mesh_drawable elbow = mesh_primitive_sphere(0.055f);

    // ------------------------------------
    mesh_drawable bec = mesh_primitive_cone(0.05f, {0,0,0}, {0,0,0.2});
    bec.uniform.color = {1,0.7, 0.3};

    // Build the hierarchy:
    // Syntax to add element
    //   hierarchy.add(visual_element, element_name, parent_name, (opt)[translation, rotation])
    hierarchy.add(body, "body");
    hierarchy.add(head, "head", "body" , radius_body * vec3(0,1,1));

    // Eyes positions are set with respect to some ratio of the
    hierarchy.add(eye, "eye_left", "head" , radius_head * vec3( 1/3.0f, 1/2.0f, 1/1.5f));
    hierarchy.add(eye, "eye_right", "head", radius_head * vec3(-1/3.0f, 1/2.0f, 1/1.5f));
    
    hierarchy.add(bec, "bec", "head", radius_head * vec3(0, 0, 1.0f));
    hierarchy.add(wing, "wing_left", "body", {-radius_body*0.75,0,0});
    hierarchy.add(wing, "wing_right", "body", {{radius_body*0.75,0,0},{-1,0,0, 0,1,0, 0,0,1}});

    hierarchy.add(wing2, "wing_left_up", "wing_left", {-length_wing,0,0});
    hierarchy.add(wing2, "wing_right_up", "wing_right", {-length_wing,0,0});

  /*  // Set the left part of the body arm: shoulder-elbow-arm
    hierarchy.add(shoulder, "shoulder_left", "body", {-radius_body+0.05f,0,0}); // extremity of the spherical body
    hierarchy.add(elbow, "elbow_left", "shoulder_left", {-length_arm,0,0});     // place the elbow the extremity of the "shoulder cylinder"
    hierarchy.add(arm, "arm_bottom_left", "elbow_left");                        // the arm start at the center of the elbow

    // Set the right part of the body arm: similar to the left part excepted a symmetry is applied along x direction for the shoulder
    hierarchy.add(shoulder, "shoulder_right", "body",     {{radius_body-0.05f,0,0}, {-1,0,0, 0,1,0, 0,0,1} } ); //Symmetry
    hierarchy.add(elbow, "elbow_right", "shoulder_right", {-length_arm,0,0});
    hierarchy.add(arm, "arm_bottom_right", "elbow_right");
*/

    // Set the same shader for all the elements
    hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    timer.scale = 0.5f;
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();

    // Current time
    const float t = timer.t;

    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the elements **/
    /** *************************************************************  **/

    // The body oscillate along the z direction
    //hierarchy["body"].transform.translation = {0,0,0.2f*(1+std::sin(2*3.14f*t))};

    // Rotation of the shoulder around the y axis
    //mat3 const R_shoulder = rotation_from_axis_angle_mat3({0,1,0}, std::sin(2*3.14f*(t-0.4f)) );
    // Rotation of the arm around the y axis (delayed with respect to the shoulder)
    //mat3 const R_arm = rotation_from_axis_angle_mat3({0,1,0}, std::sin(2*3.14f*(t-0.6f)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
   /* hierarchy["shoulder_left"].transform.rotation = R_shoulder;
    hierarchy["arm_bottom_left"].transform.rotation = R_arm;

    hierarchy["shoulder_right"].transform.rotation = Symmetry*R_shoulder; // apply the symmetry
    hierarchy["arm_bottom_right"].transform.rotation = R_arm; //note that the symmetry is already applied by the parent element
*/
    mat3 const R_wing = rotation_from_axis_angle_mat3({0,0,1}, std::sin(4*3.14f*(t-0.4f)));
    mat3 const R_wing_up = rotation_from_axis_angle_mat3({0,0,1}, std::sin(4*3.14f*(t-0.4f)));
    hierarchy["wing_left"].transform.rotation = R_wing;
    hierarchy["wing_left_up"].transform.rotation = R_wing_up;
    hierarchy["wing_right"].transform.rotation = Symmetry * R_wing;
    hierarchy["wing_right_up"].transform.rotation = R_wing_up;
    
    mat3 const R_head = rotation_from_axis_angle_mat3({1,0,0}, 0.2*std::sin(2*3.14f*t));
    hierarchy["head"].transform.rotation = R_head;
    
    hierarchy.update_local_to_global_coordinates();


    /** ********************* **/
    /** Display the hierarchy **/
    /** ********************* **/

    if(gui_scene.surface) // The default display
        draw(hierarchy, scene.camera);

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
        draw(hierarchy, scene.camera, shaders["wireframe"]);

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(hierarchy, scene.camera);

}


void scene_model::set_gui()
{
    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe); ImGui::SameLine();
    ImGui::Checkbox("Surface", &gui_scene.surface);     ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);   ImGui::SameLine();

    ImGui::Spacing();
    ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max);
    ImGui::SliderFloat("Time scale", &timer.scale, 0.1f, 3.0f);

}





#endif

