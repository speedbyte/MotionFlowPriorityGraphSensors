#include <iostream>
#include <Windows.h>
#include <glfw.h>
#include "Uni.h"

#define X 1
#define O 2

#pragma comment(lib, "glfw.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "gdi32.lib")

using namespace std;

float render();
void stepGame(float);
void keyboard(int, int);

int main(int argv, int *argc[])
{
    glfwInit();
    glfwOpenWindow(480, 480, 16, 16, 16, 16, 16, 16, GLFW_WINDOW);
    glfwSetKeyCallback(keyboard);
    glfwSetWindowTitle("Tic Tac Toe!");
    glClearColor(1.0, 1.0, 1.0, 1.0);
    float dT;

    while(glfwGetWindowParam(GLFW_OPENED) > 0)
    {
        glfwPollEvents();
        dT = render();
        stepGame(dT);
    }
    return 0;
}

#include <core/Core.h>
#include <dlfcn.h>
#include <opencv2/opencv.hpp>

// image
#include "Uni_Tuebingen_Neue_Aula.h"
cv::Mat3b image(aula.height,aula.width,(cv::Vec3b*)aula.pixel_data);

using namespace vtd_framework::core;


int main(int argc, char** argv) {
    cv::Mat* out, outsz;

    dl = dlopen(argv[1], RTLD_NOW);

    cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
    cv::namedWindow("Input");
    cv::imshow("Input",image);
    cv::waitKey(0);

    out = (cv::Mat*) pipeline->output(0)->data();

    cv::resize(*out,outsz,cv::Size(aula.width,aula.height));

    cv::namedWindow("Output");
    cv::imshow("Output",outsz);
    while(cv::waitKey(1000));
