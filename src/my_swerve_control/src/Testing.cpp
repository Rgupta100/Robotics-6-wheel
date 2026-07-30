#include <cmath>
#include <vector>
#include <iostream>

float wheel_pos_x[6] = { 0.3,  0.0, -0.3,   0.3,  0.0, -0.3}; 
float wheel_pos_y[6] = { 0.2,  0.2,  0.2,  -0.2, -0.2, -0.2}; 

float Output_Rotation[6] = {0,0,0,0,0,0};
float Output_Velocity[6] = {0,0,0,0,0,0};
/*float Last_Output_Rotation[6] = {0,0,0,0,0,0};

float Get_Current_Module_Angle(int i) {
    return Last_Output_Rotation[i]; 
}

void normalise_speeds() {
    float max_speed = 0.0;
    float limit = 1.0; // Your max allowable motor speed

    for (int i = 0; i < 6; i++) {
        if (std::abs(Output_Velocity[i]) > max_speed) {
            max_speed = std::abs(Output_Velocity[i]);
        }
    }

    if (max_speed > limit) {
        for (int i = 0; i < 6; i++) {
            Output_Velocity[i] = Output_Velocity[i]/max_speed;
        }
    }
}

void singularity(int i, float optimized_velocity, float optimized_rotation) {
    if (std::abs(optimized_velocity) > 0.001) {
        Output_Rotation[i] = optimized_rotation;
        Last_Output_Rotation[i] = Output_Rotation[i];
    } 
    else {
        Output_Rotation[i] = Last_Output_Rotation[i];
        Output_Velocity[i] = 0;
    }
}

void optimize_module(int i, float target_angle, float target_velocity) {

    float current_angle = Get_Current_Module_Angle(i);
    float delta = target_angle - current_angle;

    // 2. Wrap the delta to [-180, 180]
    while (delta > 180) delta = delta - 360;
    while (delta < -180) delta = delta + 360;

    float proposed_rotation;
    float proposed_velocity;

    if (std::abs(delta) > 90) {
        if (target_angle > 0) {
            proposed_rotation = target_angle - 180;
        } 
        else {
            proposed_rotation = target_angle + 180;
        }
        proposed_velocity = -target_velocity; 
    } 
    else {
        proposed_rotation = target_angle;
        proposed_velocity = target_velocity;
    }
    singularity(i, proposed_velocity, proposed_rotation);
}
*/
void swerve_any_pivot(float Vx, float Vy, float omega, float px, float py){

    for(int i = 0; i < 6; i++){

        float dx = wheel_pos_x[i] - px;
        float dy = wheel_pos_y[i] - py;

        float vx = Vx - omega * dy;
        float vy = Vy + omega * dx;

        Output_Rotation[i] = atan2(vy, vx) * 180 / M_PI;
        Output_Velocity[i] = sqrt(vx*vx + vy*vy);

        float raw_target_angle = atan2(vy, vx) * 180 / M_PI;
        float raw_target_velocity = sqrt(vx * vx + vy * vy);

        //optimize_module(i, raw_target_angle, raw_target_velocity);
    }
        //normalise_speeds();
}


int main(){
    swerve_any_pivot(0,0,1,0,0);
    for (int i = 0; i < 6; i++){
        std::cout<< "wheel "<<i<<" | ";
        std::cout<< "Velocity "<<Output_Velocity[i]<<" | ";
        std::cout<< "Rotation "<<Output_Rotation[i]<<" | "<<std::endl;

    }
    return 0;
}

    /*
    void angled_foreward(float velocity, float angle){
        for (int i = 0; i < 6; i++){
            Output_Rotation[i] = angle; 
            Output_Velocity[i] = velocity;
        }
    }

    void COM_turn(float velocity, float direction){

        for (int i = 0; i < 6; i++){

            float vx = -velocity * direction * wheel_pos_y[i];
            float vy =  velocity * direction * wheel_pos_x[i];

            Output_Rotation[i] = atan2(vy, vx) * 180 / M_PI;
            Output_Velocity[i] = sqrt(vx*vx + vy*vy);
        }
    }

    void swerve(float Vx, float Vy, int direction, float rotation_speed){

        float omega = direction * rotation_speed;

        for(int i=0;i<6;i++){

            float vx = Vx - omega * wheel_pos_y[i];
            float vy = Vy + omega * wheel_pos_x[i];

            Output_Rotation[i] = atan2(vy,vx) * 180 / M_PI;
            Output_Velocity[i] = sqrt(vx*vx + vy*vy);
        }
    }
    */