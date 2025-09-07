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

        det_pub_ = this->create_publisher<std_msgs::msg::String>("/signal", 10);

        cv::namedWindow("Yolo res");
        first_ = true;
        hef_file_ = "/home/hola/jazzy_ws/src/cpp_camera/yolos/puzzlebot.hef";

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

        in_bytes_ = vstreams_.first[0].get_frame_size();     // e.g. 320*320*3
        host_in_buf_.resize(in_bytes_);

        Yolo11::print_net_banner(vstreams_);

        auto activated_network_group = network_group_->activate();
        if (!activated_network_group) {
            RCLCPP_ERROR(this->get_logger(), "Error al avtivar grupo de red, i: %d, o: %d.", input_vstreams.status(), output_vstreams.status());
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
                RCLCPP_ERROR(this->get_logger(), "DmaMappedBuffer::create failed: %d", mapped.status());
                rclcpp::shutdown();
            }
            dma_in_ = std::make_unique<DmaMappedBuffer>(std::move(mapped.value()));
        }
    }

    ~Yolo11() {
        cv::destroyWindow("Yolo res");
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
        RCLCPP_INFO(
            this->get_logger(),
                    "-I---------------------------------------------------------------------"
        );
        RCLCPP_INFO(this->get_logger(), "-I- Dir   Name");
        RCLCPP_INFO(
            this->get_logger(),"-I---------------------------------------------------------------------"
        );


        for (auto &value : vstreams.first)
        {
            RCLCPP_INFO(
                this->get_logger(),
                        "-I- IN:  %s",
                        Yolo11::info_to_str<InputVStream>(value).c_str()
            );
        }

        for (auto &value : vstreams.second)
        {
            RCLCPP_INFO(
                this->get_logger(),
                        "-I- OUT: %s",
                        Yolo11::info_to_str<OutputVStream>(value).c_str()
            );
        }

        RCLCPP_INFO(
            this->get_logger(),
                    "-I---------------------------------------------------------------------"
        );
    }

    template <typename T=InputVStream>
    std::string info_to_str(T &stream)
    {
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

    template <typename T, typename A>
    int argmax(std::vector<T, A> const& vec) {
        return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
    }

    std::string classification_post_process(std::vector<float32_t>& logits, float threshold=0.65) {
        int max_idx = Yolo11::argmax(logits);
        std::vector<float32_t> softmax_result(logits);
        if (softmax_result[max_idx] < threshold) return "N\\A";
        return  "(" + std::to_string(softmax_result[max_idx]) + ")";
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
                RCLCPP_ERROR(this->get_logger(), "Unexpected input size: %zu vs %zu",
                             frame.total()*frame.channels(), in_bytes_);
                return HAILO_INVALID_ARGUMENT;
            }

            std::memcpy(host_in_buf_.data(), frame.data, in_bytes_);

            input_status = input[0].write(MemoryView(host_in_buf_.data(), in_bytes_));
            if (HAILO_SUCCESS != input_status) {
                RCLCPP_ERROR(this->get_logger(), "write() failed: %d", input_status);
                return input_status;
            }

            const auto bytes = output[0].get_frame_size();
            std::vector<float32_t> data(bytes / sizeof(float32_t));

            output_status = output[0].read(MemoryView(reinterpret_cast<uint8_t*>(data.data()), bytes));
            if (HAILO_SUCCESS != output_status) {
                RCLCPP_ERROR(this->get_logger(), "read() failed: %d", output_status);
                return output_status;
            }

            // ---- Parse Hailo NMS head ----
            const int NUM_CLASSES = 9;
            const int MAX_DETS = 100;
            const int DET_STRIDE = 5;

            if (data.size() < static_cast<size_t>(NUM_CLASSES)) {
                RCLCPP_WARN(this->get_logger(), "NMS tensor too small: %zu", data.size());
                return HAILO_INTERNAL_FAILURE;
            }

            std::array<int, NUM_CLASSES> counts{};
            for (int c = 0; c < NUM_CLASSES; ++c) {
                const float raw = (c < static_cast<int>(data.size())) ? data[c] : 0.0f;
                int cnt = static_cast<int>(std::round(raw));
                cnt = std::max(0, std::min(MAX_DETS, cnt));
                counts[c] = cnt;
            }

            size_t det_base = NUM_CLASSES;                 // offset 9
            size_t per_class_span = MAX_DETS * DET_STRIDE; // 500

            struct Det { int cls; float x, y, w, h, s; };
            std::vector<Det> kept;
            kept.reserve(32);

            for (int c = 0; c < NUM_CLASSES; ++c) {
                int cnt = counts[c];
                if (cnt <= 0) continue;

                size_t base = det_base + static_cast<size_t>(c) * per_class_span;

                for (int j = 0; j < cnt; ++j) {
                    size_t off = base + static_cast<size_t>(j) * DET_STRIDE;
                    if (off + 4 >= data.size()) break;

                    float s = data[off + 0];
                    float y = data[off + 1];
                    float w = data[off + 2];
                    float h = data[off + 3];
                    float x = data[off + 4];

                    if (s >= conf_thresh_) {
                        kept.push_back({c, x, y, w, h, s});
                    }
                }
            }

            std::sort(kept.begin(), kept.end(), [](const Det &a, const Det &b){ return a.s > b.s; });

            // Build message
            std::ostringstream msg_ss;
            msg_ss.setf(std::ios::fixed); msg_ss << std::setprecision(6);
            if (kept.empty()) {
                msg_ss << "NONE";
            } else {
                for (const auto &d : kept) {
                    const char *name = (d.cls >= 0 && d.cls < static_cast<int>(LABELS.size())) ? LABELS[d.cls] : "unknown";
                    msg_ss << name << " " << d.s << " " << d.x << " " << d.y << " " << d.w << " " << d.h << "\n";
                }
            }

            std_msgs::msg::String out_msg;
            out_msg.data = msg_ss.str();
            det_pub_->publish(out_msg);

            // Prefer iostream-style logging to avoid printf-type mismatches
            RCLCPP_INFO(this->get_logger(), "Detecciones publicadas (>= %.2f): %zu", static_cast<double>(conf_thresh_), kept.size());

            return HAILO_SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in infer(): %s", e.what());
            return HAILO_INTERNAL_FAILURE;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception in infer()");
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

        cv::Mat output(size, size, src.type(), cv::Scalar(114,114,114)); // black canvas
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
    float conf_thresh_ = 0.20f;

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
