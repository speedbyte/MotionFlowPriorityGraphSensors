
#include <core/Core.h>
#include <dlfcn.h>
#include <opencv2/opencv.hpp>

// image
#include "Uni.h"
cv::Mat3b image(aula.height,aula.width,(cv::Vec3b*)aula.pixel_data);

using namespace vtd_framework::core;

class ImagePort : public IPort {
public:
    void process(time_t time) override { }
    time_t time() override { return -1; }
    const char *name() override { return "Image"; }
    size_t size() override { return sizeof(image); }
    void *data() override { return &image; }

    void data(void *ptr) override {}
    IPort *connect() override {return(nullptr);}
    void connect(IPort *port) override {}
    IProcessable *dependency() override {return(nullptr);}
    INode *node() override {return(nullptr);}
    bool eof() override {return(false);}
    float fps() override {return(0.0f);}
    void name(const char *name) {}
};

int main(int argc, char** argv) {
    INode* pipeline;
    new_pipeline_fun_ptr new_pipeline;
    void* dl;
    cv::Mat* out, outsz;
    ImagePort port;

    sserr << sscond(argc != 2) << "expect path to library as command line argument:\t\tloadpipeline <path to protobuild library>" << ssthrow;

    dl = dlopen(argv[1], RTLD_NOW);
    sserr << sscond(!dl) << "cannot load shared library " << argv[1] << ": " << dlerror() << ssthrow;

    new_pipeline = (new_pipeline_fun_ptr) dlsym(dl, "new_pipeline");
    sserr << sscond(!new_pipeline) << "cannot find entry point of shared library " << argv[1] << ":" << dlerror() << ssthrow;

    pipeline = new_pipeline();
    sserr << sscond(!dl) << "error while creating pipeline" << ssthrow;

    cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
    cv::namedWindow("Input");
    cv::imshow("Input",image);
    cv::waitKey(0);

    pipeline->input(0)->connect(&port);
    pipeline->output(0)->process(-1);
    out = (cv::Mat*) pipeline->output(0)->data();

    cv::resize(*out,outsz,cv::Size(aula.width,aula.height));

    cv::namedWindow("Output");
    cv::imshow("Output",outsz);
    while(cv::waitKey(1000));
}

