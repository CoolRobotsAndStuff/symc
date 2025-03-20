#define CADIGO_IMPLEMENTATION
#include "../cadigo/src/cadigo.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define RGFW_IMPLEMENTATION
#include "RGFW.h"
#include <math.h>
#define DEG2RAD 3.14/180.0



static inline void cad_viz_glPerspective(double fovY, double aspect, double zNear, double zFar) {
    const double f = 1 / (cos(fovY) * sin(fovY));
    float projectionMatrix[16] = {0};
    
    projectionMatrix[0] = f / aspect;
    projectionMatrix[5] = f;
    projectionMatrix[10] = (zFar + zNear) / (zNear - zFar);
    projectionMatrix[11] = -1.0;
    projectionMatrix[14] = (2.0 * zFar * zNear) / (zNear - zFar);
    
    glMultMatrixf(projectionMatrix);
}


#define PLANET_RES 2
#define PLANET_COUNT 500

#define MAX_X 500.0L
#define MAX_Y 500.0L
#define MAX_Z 100.0L

#define MAX_RADIOUS 10.0L

#define MAX_DENSITY 200000000
#define MIN_DENSITY 10

#define MAX_VELOCITY 0.007

#define randval() ((val_t)rand()/(val_t)RAND_MAX)

#include <math.h>

typedef struct {
    bool active;
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    Vec3 color;
    val_t radious;
    val_t mass;
} Planet;

#define G 6.67430e-11 

#define CAM_VELOCITY 1

int main() {
    srand(22389238);

    Planet planets[PLANET_COUNT];

    CAD base_planet = cad_cube(1);
    for (int i = 0; i < PLANET_RES; ++i) cad_catmull_clark(&base_planet);

    val_t radious = randval() * MAX_RADIOUS;
    val_t density = (MAX_DENSITY-MIN_DENSITY) * randval() +  MIN_DENSITY;
    
    for (int i = 0; i < PLANET_COUNT; ++i) {
        planets[i].active = true;
        planets[i].position.x = randval() * MAX_X;
        planets[i].position.y = randval() * MAX_Y;
        planets[i].position.z = randval() * MAX_Z;

        planets[i].velocity.x = (randval()-0.5L)*2*MAX_VELOCITY;
        planets[i].velocity.y = (randval()-0.5L)*2*MAX_VELOCITY;
        planets[i].velocity.z = (randval()-0.5L)*2*MAX_VELOCITY;

        planets[i].acceleration = vec3(0, 0, 0);

        planets[i].color.x = randval();
        planets[i].color.y = randval();
        planets[i].color.z = randval();

        planets[i].radious = radious;

        planets[i].mass = radious * density;
    }

    // rendering:

    float pitch=31.0, yaw=230.0;
    float camX=-200, camZ=-100, camY=-200;

    RGFW_window* win = RGFW_createWindow("Cadigo Visualizer", RGFW_RECT(0, 0, 800, 450), RGFW_windowCenter | RGFW_windowNoResize );

    RGFW_window_showMouse(win, 0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glMatrixMode(GL_PROJECTION);
    glClearColor(0.0f, 0.0f, 0.05f, 1.0f);
    glLoadIdentity();
    cad_viz_glPerspective(60, 16.0 / 9.0, 1, 1000000);
    glMatrixMode(GL_MODELVIEW);

    val_t fps;
    val_t time_warping = 1000;
    val_t dt = 1/60 * time_warping;

    RGFW_window_mouseHold(win, RGFW_AREA(win->r.w / 2, win->r.h / 2));    
    while (RGFW_window_shouldClose(win) == 0) {
        while (RGFW_window_checkEvent(win)) {
            if (win->event.type == RGFW_quit) goto close_and_return;

            switch (win->event.type) {
                case RGFW_mousePosChanged: {
                    int dev_x = win->event.vector.x;
                    int dev_y = win->event.vector.y;
                    yaw   += (float)dev_x / 15.0;
                    pitch += (float)dev_y / 15.0;
                    break;
                }
                case RGFW_keyPressed:
                    switch (win->event.key) {
                        case RGFW_return:
                            RGFW_window_showMouse(win, 0);
                            RGFW_window_mouseHold(win, RGFW_AREA(win->r.w / 2, win->r.h / 2));    
                            break;

                        case RGFW_backSpace:
                            RGFW_window_showMouse(win, 1);
                            RGFW_window_mouseUnhold(win);
                            break;

                        case RGFW_q:
                            goto close_and_return;
                            break;

                        case RGFW_left:   yaw -= 5; break;
                        case RGFW_right:  yaw += 5; break;
                        case RGFW_up:   pitch -= 5; break;
                        case RGFW_down: pitch += 5; break;
                        default: break;
                    }
                    break;
                default:
                    break;
            }
        }

        if (RGFW_isPressed(win, RGFW_w)) {
            camX += cos((yaw + 90) * DEG2RAD)*CAM_VELOCITY;
            camZ -= sin((yaw + 90) * DEG2RAD)*CAM_VELOCITY;
        }
        if (RGFW_isPressed(win, RGFW_s)) {
            camX += cos((yaw + 270) * DEG2RAD)*CAM_VELOCITY;
            camZ -= sin((yaw + 270) * DEG2RAD)*CAM_VELOCITY;
        }
        if (RGFW_isPressed(win, RGFW_a)) {
            camX += cos(yaw * DEG2RAD)*CAM_VELOCITY;
            camZ -= sin(yaw * DEG2RAD)*CAM_VELOCITY;
        }
        if (RGFW_isPressed(win, RGFW_d)) {
            camX += cos((yaw + 180) * DEG2RAD)*CAM_VELOCITY;
            camZ -= sin((yaw + 180) * DEG2RAD)*CAM_VELOCITY;
        }

        if (RGFW_isPressed(win, RGFW_space))  camY -= CAM_VELOCITY;
        if (RGFW_isPressed(win, RGFW_shiftL)) camY += CAM_VELOCITY;
        
        float rot_sensitivity = 1;
        if (RGFW_isPressed(win, RGFW_h)) yaw   -= rot_sensitivity;
        if (RGFW_isPressed(win, RGFW_l)) yaw   += rot_sensitivity;
        if (RGFW_isPressed(win, RGFW_j)) pitch += rot_sensitivity;
        if (RGFW_isPressed(win, RGFW_k)) pitch -= rot_sensitivity;


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        // Update camera
        if      (pitch >=  70) pitch = 70;
        else if (pitch <= -60) pitch = -60;
        glRotatef(pitch, 1.0, 0.0, 0.0);
        glRotatef(yaw  , 0.0, 1.0, 0.0); 
        glTranslatef(camX, camY, -camZ);


        // Update planets
        for (size_t index = 0; index < PLANET_COUNT; ++index) {
            if (!planets[index].active) continue;
            Planet* planet = &planets[index];

            Vec3 total_force = vec3(0, 0, 0);
            for (size_t other_index = 0; other_index < PLANET_COUNT; ++other_index) {
                Planet* other_planet = &planets[other_index];
                if (other_index == index) continue;
                if (!other_planet->active) continue;

                Vec3 force = vec3(0, 0, 0);

                Vec3 difference = vec3_sub(other_planet->position, planet->position);

                val_t square_distance = difference.x * difference.x
                                      + difference.y * difference.y
                                      + difference.z * difference.z;

                val_t distance = sqrt(square_distance);

                //printf("%LF\n", distance);
                
                bool planets_collided = distance < (planet->radious + other_planet->radious);
                //if (planets_collided) printf("collided\n");
                //if (!planets_collided) printf("didn't collide\n");
                if (planets_collided) {
                    val_t wsum = planet->mass + other_planet->mass;
                    val_t weight1 = planet->mass / wsum;
                    val_t weight2 = other_planet->mass / wsum;
                    
                    other_planet->active = false;
                    planet->mass += other_planet->mass;

                    vec3_mult_by_s(&planet->color, weight1);
                    vec3_add_to(&planet->color, vec3_mult_s(other_planet->color, weight2));

                    vec3_mult_by_s(&planet->color, 1/vec3_max(planet->color));

                    vec3_mult_by_s(&planet->position, weight1);
                    vec3_add_to(&planet->position, vec3_mult_s(other_planet->position, weight2));

                    vec3_mult_by_s(&planet->velocity, weight1);
                    vec3_add_to(&planet->velocity, vec3_mult_s(other_planet->velocity, weight2));

                    planet->radious = max(planet->radious, other_planet->radious);
                    continue;
                }

                if (square_distance == 0.0L) continue;

                val_t magnitude = G * (planet->mass * other_planet->mass) / square_distance;


                Vec3 gravitational_force = vec3(magnitude * difference.x / distance,
                                                magnitude * difference.y / distance,
                                                magnitude * difference.z / distance);

                vec3_add_to(&total_force, gravitational_force);
            }
            vec3_div_by_s(&total_force, PLANET_COUNT);

            Vec3 acceleration = vec3_div_s(total_force, planets[index].mass);
            vec3_mult_by_s(&acceleration, (val_t)dt);

            vec3_add_to(&planets[index].velocity, acceleration);

            vec3_add_to(&planets[index].position, vec3_mult_s(planets[index].velocity, (val_t)dt));
        }

        glViewport(0, 0, win->r.w, win->r.h);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        for (size_t h = 0; h < PLANET_COUNT; ++h) {
            if (!planets[h].active) continue;
            CAD obj = cad_clone(base_planet); 
            cad_scale_s(&obj, planets[h].radious*2);
            cad_translate(&obj, planets[h].position);
            for (size_t i = 0; i < obj.faces.count; ++i) {
                Face face = obj.faces.items[i];
                if (face.count < 3) continue;  // Skip invalid faces
                Vec3 normal = cad_calculate_face_normal(obj, i);

                Vec3 difference = vec3_sub(planets[h].position, vec3(camX, camY, camZ));

                val_t square_distance = difference.x * difference.x
                                      + difference.y * difference.y
                                      + difference.z * difference.z;

                float distance = sqrt(square_distance);

                float ambient_light = 1.5;
                float brightness = 0.20;

                float lighting = (normal.z+ambient_light) * brightness * max(1.0-(distance*0.00001), 0.1);
                glBegin(GL_POLYGON);
                    glColor3f(planets[h].color.x * lighting,
                              planets[h].color.y * lighting,
                              planets[h].color.z * lighting);

                    for (size_t j = 0; j < face.count; ++j) {
                        Vec3 v = obj.points.items[face.items[j]];
                        glVertex3f(v.x, v.y, v.z);
                    }
                glEnd();
            }
            cad_free(obj);
        }

        RGFW_window_swapBuffers(win);
        fps = RGFW_window_checkFPS(win, 60);
        if (fps > 0) dt = (1/fps) * time_warping; 
    }

close_and_return:

    RGFW_window_close(win);
    return 0;
}
