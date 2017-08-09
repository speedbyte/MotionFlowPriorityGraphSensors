//
// Created by geislerd on 12.04.17.
//

#include <unused/eyetribe/VideoReader.h>

namespace vtd_framework {
    namespace eyetribe {

        class Hanshsake {
        public:
            typedef std::vector<void*> list_t;
        private:
            std::mutex m_mutex;
            list_t m_o;
            int m_i;

            void lck() {
                this->m_mutex.lock();
            }

            void ulck() {
                this->m_mutex.unlock();
            }

            template<typename T>
            T ulck(T v) {
                this->ulck();
                return v;
            }

            bool has(void* o) {
                for(list_t::iterator i = this->m_o.begin(); i != this->m_o.end(); i++)
                    if(*i == o)
                        return true;
                return false;
            }

            void *wtf() {
                if(this->m_i >= this->m_o.size())
                    this->m_i = 0;
                if(this->m_i >= this->m_o.size())
                    return nullptr;
                return this->m_o[this->m_i];
            }


        public:
            Hanshsake() : m_i(0) { }

            void reg(void* o) {
                this->lck();
                if(!this->has(o))
                    this->m_o.push_back(o);
                this->ulck();
            }

            void shk(void* o) {
                void* e;
                while(true) {
                    this->lck();
                    e = this->wtf();
                    if(e == o) {
                        this->m_i++;
                        this->ulck();
                        return;
                    }
                    this->ulck();
                    usleep(1);
                }
            }

        } handshake;

        template<uint32_t _format>
        VideoReader<_format>::VideoReader(uint32_t device) : m_device(device), m_led_code(0x000f) {
            handshake.reg(this);


            this->template output<0>()->name("image");
            this->template output<0>()->value(nullptr);

            this->reset();
        }

        template<uint32_t _format>
        VideoReader<_format>::~VideoReader() {
            uvc_stream_stop(this->m_uvc_stream_handle);
            uvc_stream_close(this->m_uvc_stream_handle);
            uvc_close(this->m_uvc_device_handle);
        }

        template<uint32_t _format>
        void VideoReader<_format>::peek(VideoReader::Image **img) {
            *img = nullptr;
            this->m_mutex.lock();
            if(!this->m_queue.empty())
                *img = this->m_queue.front();
            this->m_mutex.unlock();
        }

        template<uint32_t _format>
        void VideoReader<_format>::push(Image *img) {
            this->m_mutex.lock();
            this->m_queue.push(img);
            this->m_mutex.unlock();
        }

        template<uint32_t _format>
        void VideoReader<_format>::pop(VideoReader::Image **img) {
            this->peek(img);
            if(*img != nullptr) {
                this->m_mutex.lock();
                this->m_queue.pop();
                this->m_mutex.unlock();
            }
        }

        template<uint32_t _format>
        void VideoReader<_format>::grab(struct uvc_frame *frame) {
            const cv::Mat2b mat(HEIGHT,WIDTH,(cv::Vec2b*)frame->data);
            Image* img;

            if(frame->data_bytes != WIDTH*HEIGHT*sizeof(cv::Vec2b)) {
                sserr << sscond(this->m_uvc_error < 0) << "error while grabbing frame: "
                      << uvc_strerror(this->m_uvc_error) << ssthrow;
                return;
            }

            img = new Image();
            cv::cvtColor(mat,*img,CV_YUV2GRAY_YUYV,CV_8UC1);
            cv::flip(*img,*img,1);
            this->push(img);
        }

        template<uint32_t _format>
        void uvcstreamcallback(struct uvc_frame *frame, void *user_ptr) {
            ((VideoReader<_format>*)user_ptr)->grab(frame);
            handshake.shk(user_ptr);
        }

        template<uint32_t _format>
        void VideoReader<_format>::calc() {
            double mean_a, mean_b, mean_d;
            Image *imgs[2], *t;
            int i = 0;

            imgs[0] = this->template output<0>()->value();

            while(true) {
                this->pop(&imgs[1]);
                if(imgs[1] == nullptr)
                    usleep(1);
                else
                    break;
            }

            this->m_mutex.lock();
            std::cout << "queue: " << this->m_queue.size() << std::endl;
            this->m_mutex.unlock();

            if(imgs[0] != nullptr && imgs[0] != imgs[1])
                delete(imgs[0]);

            this->template output<0>()->value(imgs[1]);

return;
            /*
            cv::Mat1b blur;
            cv::blur(this->m_output,blur,cv::Size(51,51));

            mean_a = this->properties()->template get<float>("mean",200);
            cv::minMaxLoc(blur, nullptr,&mean_b);
            //mean_b = cv::mean(this->m_output).val[0];
            mean_d = mean_a - mean_b;


            std::cout << "mean(" << m_device << "): " << mean_d << std::endl;

            if(mean_d < -2.0f) {
                uvc_get_exposure_abs(this->m_uvc_device_handle,&this->m_cur_exposure,UVC_GET_CUR);
                if(this->m_cur_exposure > this->m_min_exposure) {
                    this->m_cur_exposure--;
                    uvc_set_exposure_abs(this->m_uvc_device_handle,this->m_cur_exposure);
                }else {
                    uvc_get_gain(this->m_uvc_device_handle,&this->m_cur_gain,UVC_GET_CUR);
                    if(this->m_cur_gain > this->m_min_gain) {
                        this->m_cur_gain--;
                        uvc_set_gain(this->m_uvc_device_handle, this->m_cur_gain);
                    }
                }
            } else if(mean_d > 2.0f) {
                uvc_get_gain(this->m_uvc_device_handle,&this->m_cur_gain,UVC_GET_CUR);
                if(this->m_cur_gain < this->m_max_gain) {
                    this->m_cur_gain++;
                    uvc_set_gain(this->m_uvc_device_handle, this->m_cur_gain);
                } else {
                    uvc_get_exposure_abs(this->m_uvc_device_handle,&this->m_cur_exposure,UVC_GET_CUR);
                    if(this->m_cur_exposure < this->m_max_exposure) {
                        this->m_cur_exposure++;
                        uvc_set_exposure_abs(this->m_uvc_device_handle,this->m_cur_exposure);
                    }
                }
            }
             */
        }

        template<uint32_t _format>
        void VideoReader<_format>::reset() {
            int count;
            uint8_t setup_data[8];

            setup_data[0] = 250;
            setup_data[1] = 0;
            setup_data[2] = 240;
            setup_data[3] = 0;
            setup_data[4] = 250;
            setup_data[5] = 0;
            setup_data[6] = 240;
            setup_data[7] = 0;

            // initialize uvc context
            this->m_uvc_error = uvc_init(&this->m_uvc_context, nullptr);
            sserr << sscond(this->m_uvc_error < 0) << "cannot initialize libuvc: " << uvc_strerror(this->m_uvc_error) << ssthrow;

            // get all available devices
            this->m_uvc_error = uvc_find_devices(
                    this->m_uvc_context,
                    &this->m_uvc_device_list,
                    EYETRIBE_VENDOR_ID,
                    EYETRIBE_PRODUCT_ID,
                    EYETRIBE_SERIAL_NUMBER);
            sserr << sscond(this->m_uvc_error < 0) << "cannot retrieve device list: " << uvc_strerror(this->m_uvc_error) << ssthrow;

            // find device
            for(count = 0; this->m_uvc_device_list[count] != nullptr; count++);
            sserr << sscond(count <= this->m_device) << "device with index " << this->m_device << " not found" << ssthrow;
            this->m_uvc_device = this->m_uvc_device_list[this->m_device];

            // open device
            this->m_uvc_error = uvc_open(this->m_uvc_device,&this->m_uvc_device_handle);
            sserr << sscond(this->m_uvc_error < 0) << "cannot open device (" << this->m_device << "): " << uvc_strerror(this->m_uvc_error) << ssthrow;

            // clear device list
            uvc_free_device_list(this->m_uvc_device_list,0);

            // setup format
            this->m_uvc_error = uvc_get_stream_ctrl_format_size(
                    this->m_uvc_device_handle,
                    &this->m_uvc_control,
                    UVC_FRAME_FORMAT_YUYV,
                    WIDTH,HEIGHT,FPS);
            sserr << sscond(this->m_uvc_error < 0)
                  << "cannot create stream control for device (" << this->m_device << "): "
                  << uvc_strerror(this->m_uvc_error) << ssthrow;

            // create stream
            this->m_uvc_error = uvc_stream_open_ctrl(this->m_uvc_device_handle,&this->m_uvc_stream_handle,&this->m_uvc_control);
            sserr << sscond(this->m_uvc_error < 0)
                  << "cannot open stream for device (" << this->m_device << "): "
                  << uvc_strerror(this->m_uvc_error) << ssthrow;

            this->m_uvc_error = uvc_stream_start(this->m_uvc_stream_handle, uvcstreamcallback<_format>,this,0);
            sserr << sscond(this->m_uvc_error < 0)
                  << "cannot start stream for device (" << this->m_device << "): "
                  << uvc_strerror(this->m_uvc_error) << ssthrow;

            // reverse engineered
            uvc_set_ctrl(this->m_uvc_device_handle,3,4,(void*)setup_data,8);

            uvc_get_gain(this->m_uvc_device_handle,&this->m_min_gain,UVC_GET_MIN);
            uvc_get_gain(this->m_uvc_device_handle,&this->m_max_gain,UVC_GET_MAX);
            uvc_get_gain(this->m_uvc_device_handle,&this->m_cur_gain,UVC_GET_CUR);

            uvc_get_exposure_abs(this->m_uvc_device_handle,&this->m_min_exposure,UVC_GET_MIN);
            uvc_get_exposure_abs(this->m_uvc_device_handle,&this->m_max_exposure,UVC_GET_MAX);
            uvc_get_exposure_abs(this->m_uvc_device_handle,&this->m_cur_exposure,UVC_GET_CUR);

            uvc_set_gain(this->m_uvc_device_handle, 50);
            uvc_set_exposure_abs(this->m_uvc_device_handle, 80);
            uvc_set_ctrl(this->m_uvc_device_handle,3,3,&this->m_led_code,2);

            //uvc_stream_start(this->m_uvc_stream_handle,uvcstreamcallback<_format>,this,0);
        }

        template class VideoReader<0>;
        template class VideoReader<1>;
        template class VideoReader<2>;
        template class VideoReader<3>;
        template class VideoReader<4>;
        template class VideoReader<5>;
        template class VideoReader<6>;
        template class VideoReader<7>;
    }
}