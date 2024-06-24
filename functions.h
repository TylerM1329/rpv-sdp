#ifndef FUNCTIONS_H
#define FUNCTIONS_H

long map(long, long, long, long, long);
int run_camera_pan(int);
void run_acceleration(int, int, int);
void run_steering(int, int, int, int, int);
int setup_TCP_Server(char[], int);
void run_brake_lights(int, int, int);
void run_reverse_lights(int, int, int);
void run_turn_signals(int, int, int);
int run_active_safety(int, int);
void get_network_options(char*, int&);
void get_user_options(int&, int&);
void parse_control_data(int, char[], int&, int&, int&, int&, int&, int&);
int get_cv_flag(int, char []);
int init_IO(int&, int&, int, int&, int, int&, int&);
void run_headlights(int, int, int);
void run_cv_status_led(int, int, int);
void disable_motors(int);

#endif