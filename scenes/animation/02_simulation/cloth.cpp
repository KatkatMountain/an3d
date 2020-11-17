
#include "cloth.hpp"
#include <cmath>


#ifdef SCENE_CLOTH

using namespace vcl;


// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
//
float current_magnitude = 0.f;
void scene_model::compute_forces()
{
    const size_t N = force.size();        // Total number of particles of the cloth Nu x Nv
    const int N_dim = int(force.dimension[0]); // Number of particles along one dimension (square dimension)

    simulation_parameters.m = user_parameters.m / float(N); // Constant total mass

    // Get simuation parameters
    const float K  = user_parameters.K;
    const float m  = simulation_parameters.m;
    const float L0 = simulation_parameters.L0;

    // Gravity
    const vec3 g = {0,-9.81f,0};
    for(size_t k=0; k<N; ++k)
        force[k] = m*g;

    // Drag
    const float mu = user_parameters.mu;
    for(size_t k=0; k<N; ++k)
        force[k] = force[k]-mu*speed[k];

    // Wind
    const vec3 w = {-current_magnitude, 0, 0};
    for(int ku=0; ku<N_dim; ++ku) {
        for(int kv=0; kv<N_dim; ++kv) {
            int current = ku * N_dim + kv;
            vec3 normal {0, 0, 0};
            unsigned nb_normal = 0;
            if (ku > 0){
                int up = (ku - 1) * N_dim + kv;
                if (kv > 0){
                    int left = ku * N_dim + kv - 1;
                    vec3 temp = normalize(cross(normalize(position[up] - position[current]),
                                              normalize(position[left] - position[current])));
                    if (temp[0] > 0)
                        temp = -temp;
                    normal += temp;
                    nb_normal++;
                }
                if (kv < N_dim - 1)
                {
                    int right = ku * N_dim + kv + 1;
                    vec3 temp = normalize(cross(normalize(position[up] - position[current]),
                                              normalize(position[right] - position[current])));
                    if (temp[0] > 0)
                        temp = -temp;
                    normal += temp;
                    nb_normal++;
                }
            }
            if (ku < N_dim - 1){
                int down = (ku + 1) * N_dim + kv;
                if (kv > 0){
                    int left = ku * N_dim + kv - 1;
                    vec3 temp = normalize(cross(normalize(position[down] - position[current]),
                                              normalize(position[left] - position[current])));
                    if (temp[0] > 0)
                        temp = -temp;
                    normal += temp;
                    nb_normal++;
                }
                if (kv < N_dim - 1)
                {
                    int right = ku * N_dim + kv + 1;
                    vec3 temp = normalize(cross(normalize(position[down] - position[current]),
                                              normalize(position[right] - position[current])));
                    if (temp[0] > 0)
                        temp = -temp;
                    normal += temp;
                    nb_normal++;
                }
            }

            normal /= nb_normal;
            force[current] += w * 0.001f * dot(normalize(w), normal);
        }
    }

    // Springs
    for(int ku=0; ku<N_dim; ++ku) {
        for(int kv=0; kv<N_dim; ++kv) {
            // To do ...
            // Compute spring forces force(ku,kv) = ...
            int current = ku * N_dim + kv;

            for (int i = -2; i < 3; i++) {
                for (int j = -2; j < 3; j++)
                {
                    if ((i == 0 && j == 0) || (abs(i) == 2 && abs(j) == 1)
                        || (abs(i) == 1 && abs(j) == 2) || (abs(i) == 2 && abs(j) == 2))
                        continue;

                    if (ku+i >= 0 && ku+i < N_dim && kv+j >= 0 && kv+j < N_dim)
                    {
                        vec3 const pji = position[(ku+i) * N_dim + kv+j] - position[current];
                        float const L = norm(pji);

                        float Lij = L0; // L0 for structural springs
                        
                        if (i != 0 && j!= 0)    // L0 for shearing springs
                            Lij = sqrt(2 * L0*L0);

                        if (abs(i) == 2 || abs(j) == 2) // L0 for bending springs
                            Lij = 2 * L0;

                        force[current] = force[current] + K * (L - Lij) * pji / L;
                    }
                }
            }
        }
    }
}


// Handle detection and response to collision with the shape described in "collision_shapes" variable
void scene_model::collision_constraints()
{
    //Ground collision
    const size_t N = force.size();        // Total number of particles of the cloth Nu x Nv
    for (size_t k = 0; k < N; ++k)
    {
        if (position[k][1] < collision_shapes.ground_height)
            position[k][1] = collision_shapes.ground_height;
    }
    
    //Sphere collision
    for (size_t k = 0; k < N; ++k)
    {
        float detection = norm(position[k] - collision_shapes.sphere_p);
        if (detection <= collision_shapes.sphere_r + 0.01f)
        {
            vec3 u = (position[k] - collision_shapes.sphere_p) / norm(position[k] - collision_shapes.sphere_p);
            float m1 = 0.01f;
            float m2 = 0.01f;
            float j = 2 * (m1 * m2) / (m1 + m2) * dot(- speed[k], u);

            speed[k] = 0.1f * speed[k] + 0.1f * j/(user_parameters.m / (float)(N));

            float d = collision_shapes.sphere_r + 0.01f - norm(position[k] - collision_shapes.sphere_p);
            //speed[k] = 0.1f * speed[k];
            position[k] = position[k] + d/2.f*u;
        }
    }
}



// Initialize the geometrical model
void scene_model::initialize()
{
    // Number of samples of the model (total number of particles is N_cloth x N_cloth)
    const size_t N_cloth = 50;

    // Rest length (length of an edge)
    simulation_parameters.L0 = 1.0f/float(N_cloth-1);

    // Create cloth mesh in its initial position
    // Horizontal grid of length 1 x 1
    const mesh base_cloth = mesh_primitive_grid(N_cloth,N_cloth,{0,1,-0.5f},{1,0,0},{0,0,1});

    // Set particle position from cloth geometry
    position = buffer2D_from_vector(base_cloth.position, N_cloth, N_cloth);

    // Set hard positional constraints
    positional_constraints[0] = position[0];
    positional_constraints[N_cloth*(N_cloth-1)] = position[N_cloth*(N_cloth-1)];

    // Init particles data (speed, force)
    speed.resize(position.dimension); speed.fill({0,0,0});
    force.resize(position.dimension); force.fill({0,0,0});


    // Store connectivity and normals
    connectivity = base_cloth.connectivity;
    normals      = normal(position.data,connectivity);

    // Send data to GPU
    cloth.clear();
    cloth = mesh_drawable(base_cloth);
    cloth.uniform.shading.specular = 0.0f;
    cloth.shader = shader_mesh;
    cloth.texture_id = texture_cloth;

    simulation_diverged = false;
    force_simulation    = false;

    timer.update();
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& gui)
{
    gui.show_frame_camera = false;

    // Load textures
    texture_cloth = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/cloth.png"));
    texture_wood  = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/wood.png"));
    shader_mesh = shaders["mesh_bf"];

    // Initialize cloth geometry and particles
    initialize();

    // Default value for simulation parameters
    user_parameters.K    = 100.0f;
    user_parameters.m    = 5.0f;
    user_parameters.wind = 0.0f;
    user_parameters.mu   = 0.02f;

    // Set collision shapes
    collision_shapes.sphere_p = {0,0.1f,0};
    collision_shapes.sphere_r = 0.1f;
    collision_shapes.ground_height = 0.1f;

    // Init visual models
    sphere = mesh_drawable(mesh_primitive_sphere(1.0f,{0,0,0},60,60));
    sphere.shader = shaders["mesh"];
    sphere.uniform.color = {1,0,0};

    ground = mesh_drawable(mesh_primitive_quad({-1,collision_shapes.ground_height-1e-3f,-1}, {1,collision_shapes.ground_height-1e-3f,-1}, {1,collision_shapes.ground_height-1e-3f,1}, {-1,collision_shapes.ground_height-1e-3f,1}));
    ground.shader = shaders["mesh_bf"];
    ground.texture_id = texture_wood;

    gui_display_texture = true;
    gui_display_wireframe = false;
}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    const float dt = timer.update();
    set_gui();

    // Force constant simulation time step
    float h = dt<=1e-6f? 0.0f : timer.scale*0.001f;

    if( (!simulation_diverged || force_simulation) && h>0)
    {
        // Iterate over a fixed number of substeps between each frames
        const size_t number_of_substeps = 4;
        for(size_t k=0; (!simulation_diverged  || force_simulation) && k<number_of_substeps; ++k)
        {
            current_magnitude = user_parameters.wind + (user_parameters.wind / 2.f) * sinf(0.1*dt);
            
            compute_forces();
            numerical_integration(h);
            collision_constraints();                 // Detect and solve collision with other shapes

            hard_constraints();                      // Enforce hard positional constraints

            normal(position.data, connectivity, normals); // Update normals of the cloth
            detect_simulation_divergence();               // Check if the simulation seems to diverge
        }
    }


    cloth.update_position(position.data);
    cloth.update_normal(normals.data);

    display_elements(shaders, scene, gui);

}

void scene_model::numerical_integration(float h)
{
    const size_t NN = position.size();
    const float m = simulation_parameters.m;

    for(size_t k=0; k<NN; ++k)
    {
        vec3& p = position[k];
        vec3& v = speed[k];
        const vec3& f = force[k];

        v = v + h*f/m;
        p = p + h*v;
    }
}

void scene_model::hard_constraints()
{
    // Fixed positions of the cloth
    for(const auto& constraints : positional_constraints)
        position[constraints.first] = constraints.second;
}


void scene_model::display_elements(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    glEnable( GL_POLYGON_OFFSET_FILL );

    // Display cloth
    GLuint texture = cloth.texture_id;
    if(!gui_display_texture)
        texture = scene.texture_white;

    glPolygonOffset( 1.0, 1.0 );
    draw(cloth, scene.camera, cloth.shader, texture);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);

    if(gui_display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(cloth, scene.camera, shaders["wireframe_quads"]);
    }


    // Display positional constraint using spheres
    sphere.uniform.transform.scaling = 0.02f;
    for(const auto& constraints : positional_constraints)  {
        sphere.uniform.transform.translation = constraints.second;
        draw(sphere, scene.camera, shaders["mesh"]);
    }


    // Display sphere used for collision
    sphere.uniform.transform.scaling     = collision_shapes.sphere_r;
    sphere.uniform.transform.translation = collision_shapes.sphere_p;
    draw(sphere, scene.camera, shaders["mesh"]);

    // Display ground
    draw(ground, scene.camera);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}


// Automatic detection of divergence: stop the simulation if detected
void scene_model::detect_simulation_divergence()
{
    const size_t NN = position.size();
    for(size_t k=0; simulation_diverged==false && k<NN; ++k)
    {
        const float f = norm(force[k]);
        const vec3& p = position[k];

        if( std::isnan(f) ) // detect NaN in force
        {
            std::cout<<"NaN detected in forces"<<std::endl;
            simulation_diverged = true;
        }

        if( f>1000.0f ) // detect strong force magnitude
        {
            std::cout<<" **** Warning : Strong force magnitude detected "<<f<<" at vertex "<<k<<" ****"<<std::endl;
            simulation_diverged = true;
        }

        if( std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) ) // detect NaN in position
        {
            std::cout<<"NaN detected in positions"<<std::endl;
            simulation_diverged = true;
        }

        if(simulation_diverged==true)
        {
            std::cerr<<" **** Simulation has diverged **** "<<std::endl;
            std::cerr<<" > Stop simulation iterations"<<std::endl;
            timer.stop();
        }
    }

}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Stiffness", &user_parameters.K, 1.0f, 400.0f, "%.2f s");
    ImGui::SliderFloat("Damping", &user_parameters.mu, 0.0f, 0.1f, "%.3f s");
    ImGui::SliderFloat("Mass", &user_parameters.m, 1.0f, 15.0f, "%.2f s");
    ImGui::SliderFloat("Wind", &user_parameters.wind, 0.0f, 400.0f, "%.2f s");

    ImGui::Checkbox("Wireframe",&gui_display_wireframe);
    ImGui::Checkbox("Texture",&gui_display_texture);

    bool const stop  = ImGui::Button("Stop anim"); ImGui::SameLine();
    bool const start = ImGui::Button("Start anim");

    if(stop)  timer.stop();
    if(start) {
        if( simulation_diverged )
            force_simulation=true;
        timer.start();
    }

    bool const restart = ImGui::Button("Restart");
    if(restart) initialize();
}









#endif
