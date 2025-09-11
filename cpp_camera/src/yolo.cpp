#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <cstring>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hailo/hailort.hpp"

using hailort::Device;
using hailort::Hef;
using hailort::Expected;
using hailort::make_unexpected;
using hailort::ConfiguredNetworkGroup;
using hailort::VStreamsBuilder;
using hailort::InputVStream;
using hailort::OutputVStream;
using hailort::MemoryView;
using hailort::DmaMappedBuffer;

class Yolo11 : public rclcpp::Node {
public:
    Yolo11()
    : Node("yolo11") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/video_source/raw", 10, std::bind(&Yolo11::sub_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&Yolo11::timer_callback, this)
        );

        // Publish the single best label to /signals
        det_pub_ = this->create_publisher<std_msgs::msg::String>("/signal", 10);

        cv::namedWindow("Yolo");
        first_ = true;
        hef_file_ = (ament_index_cpp::get_package_share_directory("cpp_camera") + "/yolos/puzzlebot.hef");

        auto all_devices = Device::scan_pcie();
        auto device = Device::create_pcie(all_devices.value()[0]);
        if (!device) {
            RCLCPP_ERROR(this->get_logger(), "Error al crear dispositivo pcie: '%d'", device.status());
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Se creo dispositivo pcie: '%d'", device.status());
        }
        device_ = std::move(device.value());

        auto ng = Yolo11::configure_network_group(*device_, hef_file_);
        if (!ng) {
            RCLCPP_ERROR(this->get_logger(), "Error al configurar grupo red: '%s'", hef_file_.c_str());
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Se cofiguro grupo de red con exito: '%s'", hef_file_.c_str());
        }
        network_group_ = ng.value();

        auto input_vstream_params = network_group_->make_input_vstream_params(true, HAILO_FORMAT_TYPE_UINT8, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
        auto output_vstream_params = network_group_->make_output_vstream_params(false, HAILO_FORMAT_TYPE_FLOAT32, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
        auto input_vstreams  = VStreamsBuilder::create_input_vstreams(*network_group_, input_vstream_params.value());
        auto output_vstreams = VStreamsBuilder::create_output_vstreams(*network_group_, output_vstream_params.value());

        if (!input_vstreams or !output_vstreams) {
            RCLCPP_ERROR(this->get_logger(), "Error al crear entrada: '%d', estado de la salida '%d'", input_vstreams.status(), output_vstreams.status());
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Se creo entrada y salida con exito.");
        }

        vstreams_ = std::make_pair(input_vstreams.release(), output_vstreams.release());

        in_bytes_ = vstreams_.first[0].get_frame_size();
        host_in_buf_.resize(in_bytes_);

        Yolo11::print_net_banner(vstreams_);

        auto activated_network_group = network_group_->activate();
        if (!activated_network_group) {
            RCLCPP_ERROR(this->get_logger(), "Error al activar grupo de red, i: %d, o: %d.", input_vstreams.status(), output_vstreams.status());
            rclcpp::shutdown();
        }
        activated_group_ = std::move(activated_network_group.value());

        {
            auto mapped = hailort::DmaMappedBuffer::create(
                *device_,
                static_cast<void*>(host_in_buf_.data()),
                in_bytes_,
                HAILO_DMA_BUFFER_DIRECTION_H2D
            );
            if (!mapped) {
                RCLCPP_ERROR(this->get_logger(), "Error al crear Dma Buffer: %d", mapped.status());
                rclcpp::shutdown();
            }
            dma_in_ = std::make_unique<DmaMappedBuffer>(std::move(mapped.value()));
        }
    }

    ~Yolo11() {
        cv::destroyWindow("Yolo");
    }

private:
    void timer_callback() {
        return;
    }

    void sub_callback(sensor_msgs::msg::Image::UniquePtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::Size s = frame.size();
        if (first_) {
            RCLCPP_INFO(this->get_logger(), "Recibiendo imagenes de largo: '%d' y alto: '%d'", s.width, s.height);
            first_ = false;
        }

        auto status = Yolo11::infer<uint8_t, float32_t>(vstreams_.first, vstreams_.second, frame);

        cv::imshow("Yolo res", frame);
        cv::waitKey(1);
    }

    Expected<std::shared_ptr<ConfiguredNetworkGroup>> configure_network_group(Device &device, const std::string &hef_file) {
        auto hef = Hef::create(hef_file);
        if (!hef) {
            return make_unexpected(hef.status());
        }

        auto configure_params = hef->create_configure_params(HAILO_STREAM_INTERFACE_PCIE);
        if (!configure_params) {
            return make_unexpected(configure_params.status());
        }

        auto network_groups = device.configure(hef.value(), configure_params.value());
        if (!network_groups) {
            return make_unexpected(network_groups.status());
        }

        if (1 != network_groups->size()) {
            RCLCPP_ERROR(this->get_logger(), "Error numero de grupos de red invalidos.");
            return make_unexpected(HAILO_INTERNAL_FAILURE);
        }

        return std::move(network_groups->at(0));
    }

    void print_net_banner(std::pair< std::vector<InputVStream>, std::vector<OutputVStream> > &vstreams) {
        RCLCPP_INFO(this->get_logger(), "-I---------------------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "-I- Dir   Name");
        RCLCPP_INFO(this->get_logger(), "-I---------------------------------------------------------------------");

        for (auto &value : vstreams.first) {
            RCLCPP_INFO(this->get_logger(), "-I- IN:  %s", Yolo11::info_to_str<InputVStream>(value).c_str());
        }
        for (auto &value : vstreams.second) {
            RCLCPP_INFO(this->get_logger(), "-I- OUT: %s", Yolo11::info_to_str<OutputVStream>(value).c_str());
        }

        RCLCPP_INFO(this->get_logger(), "-I---------------------------------------------------------------------");
    }

    template <typename T=InputVStream>
    std::string info_to_str(T &stream) {
        std::string result = stream.get_info().name;
        result += " (";
        result += std::to_string(stream.get_info().shape.height);
        result += ", ";
        result += std::to_string(stream.get_info().shape.width);
        result += ", ";
        result += std::to_string(stream.get_info().shape.features);
        result += ")";
        return result;
    }

    template <typename IN_T, typename OUT_T>
    hailo_status infer(std::vector<InputVStream> &input, std::vector<OutputVStream> &output, cv::Mat frame_i) {
        try {
            hailo_status input_status = HAILO_UNINITIALIZED;
            hailo_status output_status = HAILO_UNINITIALIZED;

            cv::Mat frame;
            cv::cvtColor(frame_i, frame, cv::COLOR_BGR2RGB);
            frame = Yolo11::letterbox(frame, 320);

            if (frame.total()*frame.channels() != in_bytes_) {
                RCLCPP_ERROR(this->get_logger(), "Tamaño de frame no esperado: %zu vs %zu",
                             frame.total()*frame.channels(), in_bytes_);
                return HAILO_INVALID_ARGUMENT;
            }

            std::memcpy(host_in_buf_.data(), frame.data, in_bytes_);

            input_status = input[0].write(MemoryView(host_in_buf_.data(), in_bytes_));
            if (HAILO_SUCCESS != input_status) {
                RCLCPP_ERROR(this->get_logger(), "Fallo en escritura: %d", input_status);
                return input_status;
            }

            const auto bytes = output[0].get_frame_size();
            std::vector<float32_t> data(bytes / sizeof(float32_t));

            output_status = output[0].read(MemoryView(reinterpret_cast<uint8_t*>(data.data()), bytes));
            if (HAILO_SUCCESS != output_status) {
                RCLCPP_ERROR(this->get_logger(), "Fallo en lectura: %d", output_status);
                return output_status;
            }

            auto intish = [](float v)->bool { return std::fabs(v - std::round(v)) <= 1e-4f; };

            const int NUM_CLASSES = 9;
            size_t i = 0;
            int cur_cls = 0;

            int best_cls = -1;
            float best_score = -1.0f;

            auto safe_next = [&](float &dst)->bool {
                if (i >= data.size()) return false;
                dst = data[i++];
                return true;
            };

            while (cur_cls < NUM_CLASSES && i < data.size()) {
                while (cur_cls < NUM_CLASSES && i < data.size() &&
                    intish(data[i]) && std::round(data[i]) == 0.0f)
                {
                    ++i;
                    ++cur_cls;
                }

                if (cur_cls >= NUM_CLASSES || i >= data.size()) break;
                if (!intish(data[i])) break;

                int cnt = static_cast<int>(std::round(data[i++]));
                if (cnt < 0) cnt = 0;
                if (cnt > 200) cnt = 200;

                for (int k = 0; k < cnt; ++k) {
                    // orden y1, x1, y2, x2, score
                    float y1, x1, y2, x2, s;
                    if (!safe_next(y1) || !safe_next(x1) || !safe_next(y2) || !safe_next(x2) || !safe_next(s)) {
                        i = data.size(); break;
                    }
                    if (s >= conf_thresh_ && s > best_score) {
                        best_score = s;
                        best_cls = cur_cls;
                    }
                }
                ++cur_cls;
            }

            std_msgs::msg::String out_msg;
            if (best_cls >= 0) {
                out_msg.data = LABELS[best_cls];
                det_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "TOP: %s (%.3f)", LABELS[best_cls], static_cast<double>(best_score));
            } else {
                out_msg.data = "";
                det_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "TOP: NONE");
            }

            return HAILO_SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error en inferencia: %s", e.what());
            return HAILO_INTERNAL_FAILURE;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error desconocido en inferencia.");
            return HAILO_INTERNAL_FAILURE;
        }
    }

    cv::Mat letterbox(const cv::Mat &src, int size=320) {
        int w = src.cols, h = src.rows;
        float scale = std::min(size / (float)w, size / (float)h);
        int nw = int(w * scale);
        int nh = int(h * scale);

        cv::Mat resized;
        cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);

        cv::Mat output(size, size, src.type(), cv::Scalar(0,0,0)); // black canvas
        int top  = (size - nh) / 2;
        int left = (size - nw) / 2;
        resized.copyTo(output(cv::Rect(left, top, nw, nh)));

        return output;
    }

    static constexpr std::array<const char*, 9> LABELS = {
        "give_way", "go_straight", "green", "red", "roadwork_ahead",
        "stop", "turn_left", "turn_right", "yellow"
    };

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr det_pub_;
    float conf_thresh_ = 0.65f;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_;
    std::string hef_file_;
    std::unique_ptr<Device> device_;
    std::pair< std::vector<InputVStream>, std::vector<OutputVStream>> vstreams_;
    std::shared_ptr<ConfiguredNetworkGroup> network_group_;
    std::unique_ptr<hailort::ActivatedNetworkGroup> activated_group_;
    std::vector<uint8_t> host_in_buf_;
    std::shared_ptr<DmaMappedBuffer> dma_in_;
    size_t in_bytes_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Yolo11>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
