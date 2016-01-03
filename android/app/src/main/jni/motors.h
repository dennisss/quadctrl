#ifndef MOTORS_H_
#define MOTORS_H_

void motors_init(char *fspath, int vid, int pid, int fd);
void motors_destroy();
void motors_set(float *speeds);

double battery_level();



#endif