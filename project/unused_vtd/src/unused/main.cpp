//
// Created by veikas on 09.08.17.
//

case BOOLEANMAPS:
{
    vtd_framework::core::Utils::setMainStackSize();
    cv::destroyAllWindows();
    process_booleanmaps(argc == 4, rc);
    break;
}

case SPECTRALWHITENING:
{
    vtd_framework::core::Utils::setMainStackSize();
    cv::destroyAllWindows();
    process_spectralwhitening(argc == 4, rc);
    break;
}

void process_spectralwhitening(bool saveOutput, const vtd_framework::run_configurations::RunConfigurations &rc) {
    const uint32_t WIDTH = uint32_t(vtd_framework::kitti::LeftRGBImageReader::Image::WIDTH);
    const uint32_t HEIGHT = uint32_t(vtd_framework::kitti::LeftRGBImageReader::Image::HEIGHT);

    vtd_framework::core::Pipeline pipeline;
    vtd_framework::kitti::LeftRGBImageReader image_reader(rc.m_dataset_path);
    vtd_framework::kitti::LeftRGBImageReader::Image::ConvertLAB lab;
    vtd_framework::kitti::LeftRGBImageReader::Image::ConvertLAB::OutputImage::Splitt lab_splitt;
    vtd_framework::saliency::activation::_Spectral<WIDTH,HEIGHT> spectral_l;
    vtd_framework::saliency::activation::_Spectral<WIDTH,HEIGHT> spectral_a;
    vtd_framework::saliency::activation::_Spectral<WIDTH,HEIGHT> spectral_b;
    vtd_framework::utils::_Matrix<WIDTH,HEIGHT,cv::Vec3f>::Merge spectral_lab;
    vtd_framework::utils::_Matrix<WIDTH,HEIGHT,cv::Vec3f>::Sum spectral;
    vtd_framework::utils::_HeatmapImage<WIDTH,HEIGHT>::ConvertRGB rgb;
    vtd_framework::io::ImageShow show("Spectral Whitening");

    vtd_framework::utils::FPSCounter fps;
    vtd_framework::plot::Plot fps_plot;
    vtd_framework::io::ImageShow fps_show("FPS - AVG 10");

    std::cout << "create pipeline" << std::endl;
    connect_port(image_reader,0,lab,0);
    connect_port(lab,0,lab_splitt,0);
    connect_port(lab_splitt,0,spectral_l,0);
    connect_port(lab_splitt,1,spectral_a,0);
    connect_port(lab_splitt,2,spectral_b,0);
    connect_port(spectral_l,0,spectral_lab,0);
    connect_port(spectral_a,0,spectral_lab,1);
    connect_port(spectral_b,0,spectral_lab,2);
    connect_port(spectral_lab,0,spectral,0);
    connect_port(spectral,0,rgb,0);
    connect_port(rgb,0,show,0);

    connect_port(rgb,0,fps,0);
    connect_port(fps,0,fps_plot,0);
    connect_port(fps_plot,0,fps_show,0);

    pipeline.pushNode(&image_reader);
    pipeline.pushNode(&lab);
    pipeline.pushNode(&lab_splitt);
    pipeline.pushNode(&spectral_l);
    pipeline.pushNode(&spectral_a);
    pipeline.pushNode(&spectral_b);
    pipeline.pushNode(&spectral_lab);
    pipeline.pushNode(&spectral);
    pipeline.pushNode(&rgb);
    pipeline.pushNode(&show);
    pipeline.pushNode(&fps_show);

    pipeline.pushNode(&fps);
    pipeline.pushNode(&fps_plot);

    std::cout << "initialize pipeline" << std::endl;
    show.properties()->set<bool>("close_window",false);
    pipeline.initialize();

    std::cout << "process pipeline" << std::endl;
    for(time_t time = 0; !pipeline.eof(); time++)
        pipeline.process(time);
}

void process_booleanmaps(bool saveOutput, const vtd_framework::run_configurations::RunConfigurations &rc) {

    const uint32_t WIDTH = uint32_t(vtd_framework::kitti::LeftRGBImageReader::Image::WIDTH);
    const uint32_t HEIGHT = uint32_t(vtd_framework::kitti::LeftRGBImageReader::Image::HEIGHT);

    vtd_framework::core::Pipeline pipeline;
    vtd_framework::kitti::LeftRGBImageReader image_reader(rc.m_dataset_path);
    vtd_framework::kitti::LeftRGBImageReader::Image::ConvertLAB lab;
    vtd_framework::kitti::LeftRGBImageReader::Image::ConvertLAB::OutputImage::Splitt lab_splitt;
    vtd_framework::utils::_HeatmapImage<WIDTH,HEIGHT>::ConvertIntensity int_l;
    vtd_framework::utils::_HeatmapImage<WIDTH,HEIGHT>::ConvertIntensity int_a;
    vtd_framework::utils::_HeatmapImage<WIDTH,HEIGHT>::ConvertIntensity int_b;
    vtd_framework::saliency::activation::_BooleanMaps<WIDTH,HEIGHT> bm_l;
    vtd_framework::saliency::activation::_BooleanMaps<WIDTH,HEIGHT> bm_a;
    vtd_framework::saliency::activation::_BooleanMaps<WIDTH,HEIGHT> bm_b;
    vtd_framework::utils::_Matrix<WIDTH,HEIGHT,cv::Vec3f>::Merge bm_lab;
    vtd_framework::utils::_Matrix<WIDTH,HEIGHT,cv::Vec3f>::Sum bm;
    vtd_framework::utils::_HeatmapImage<WIDTH,HEIGHT>::ConvertRGB rgb;
    vtd_framework::io::ImageShow show("Boolean Maps");

    vtd_framework::utils::FPSCounter fps;
    vtd_framework::plot::Plot fps_plot;
    vtd_framework::io::ImageShow fps_show("FPS - AVG 10");

    std::cout << "create pipeline" << std::endl;
    connect_port(image_reader,0,lab,0);
    connect_port(lab,0,lab_splitt,0);
    connect_port(lab_splitt,0,int_l,0);
    connect_port(lab_splitt,1,int_a,0);
    connect_port(lab_splitt,2,int_b,0);
    connect_port(int_l,0,bm_l,0);
    connect_port(int_a,0,bm_a,0);
    connect_port(int_b,0,bm_b,0);
    connect_port(bm_l,0,bm_lab,0);
    connect_port(bm_a,0,bm_lab,1);
    connect_port(bm_b,0,bm_lab,2);
    connect_port(bm_lab,0,bm,0);
    connect_port(bm,0,rgb,0);
    connect_port(rgb,0,show,0);

    connect_port(rgb,0,fps,0);
    connect_port(fps,0,fps_plot,0);
    connect_port(fps_plot,0,fps_show,0);

    pipeline.pushNode(&image_reader);
    pipeline.pushNode(&lab);
    pipeline.pushNode(&lab_splitt);
    pipeline.pushNode(&bm_l);
    pipeline.pushNode(&bm_a);
    pipeline.pushNode(&bm_b);
    pipeline.pushNode(&bm_lab);
    pipeline.pushNode(&bm);
    pipeline.pushNode(&rgb);
    pipeline.pushNode(&show);
    pipeline.pushNode(&fps_show);
    pipeline.pushNode(&fps);
    pipeline.pushNode(&fps_plot);

    std::cout << "initialize pipeline" << std::endl;
    show.properties()->set<bool>("close_window",false);
    pipeline.initialize();

    std::cout << "process pipeline" << std::endl;
    for(time_t time = 0; !pipeline.eof(); time++)
        pipeline.process(time);
}//
