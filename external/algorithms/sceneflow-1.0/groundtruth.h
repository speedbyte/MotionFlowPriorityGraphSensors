#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

float disp_sphere(double x, double y, double r);
float dispx_sphere(double x, double y, double r);
float dispy_sphere(double x, double y, double r);
float flowx_sphere(double x, double y, double r, double t);
float flowy_sphere(double x, double y, double r, double t);
void recons(double x, double y, double d, double *X, double *Y, double *Z);
void proj(double X, double Y, double Z, double *x, double *y, double *d);
void flow_sphere(int x_pixels, int y_pixels, int image_width, double sphere_radius, double angle_X, double angle_Y, double angle_Z, double *flow_x, double *flow_y, double *disp);

#endif
