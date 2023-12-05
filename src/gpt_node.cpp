#include <fstream>
#include <iostream>
#include <string>
#include <queue>

//#include <curl/curl.h>

#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace gpt_node {

const std::string API_URL = "https://api.openai.com/v1/chat/completions";

struct GPTConfig {
  std::string api_key;
  bool chat_mode_enable;
  std::string model;
  std::string prompt;
  double temperature;
  int max_tokens;
  double top_p;
  double frequency_penalty;
  double presence_penalty;
};

size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                     std::string *output) {
  size_t total_size = size * nmemb;
  output->append(static_cast<char *>(contents), total_size);
  return total_size;
}

std::string SendPostRequest(const std::string &data,
                            const std::string &api_key) {
  CURL *curl = curl_easy_init();
  std::string response;

  if (curl) {
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers,
                                ("Authorization: Bearer " + api_key).c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_URL, API_URL.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
      std::cerr << "Failed to perform HTTP request: " << curl_easy_strerror(res)
                << std::endl;
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
  }

  return response;
}

std::string SendChatGPTRequest(nlohmann::ordered_json &request_message,
                               GPTConfig config) {
  nlohmann::ordered_json request_body;
  request_body["model"] = config.model;
  request_body["messages"] = request_message;
  request_body["temperature"] = config.temperature;
  request_body["max_tokens"] = config.max_tokens;
  request_body["top_p"] = config.top_p;
  request_body["frequency_penalty"] = config.frequency_penalty;
  request_body["presence_penalty"] = config.presence_penalty;

  std::string json_data = request_body.dump();
  std::string response = SendPostRequest(json_data, config.api_key);

  nlohmann::ordered_json root = nlohmann::ordered_json::parse(response);
  std::string reply = root["choices"][0]["message"]["content"];
  return reply;
}

class GPTNode : public rclcpp::Node {
public:
  GPTNode() : Node("gpt_node") {
    std::string topic_subscription_name = "/request_text";
    declare_parameter<std::string>("gpt_topic_sub", topic_subscription_name);
    get_parameter<std::string>("gpt_topic_sub", topic_subscription_name);

    subscription_ = create_subscription<std_msgs::msg::String>(
        topic_subscription_name, rclcpp::QoS(10),
        std::bind(&GPTNode::InputCallback, this, std::placeholders::_1));

    std::string topic_publisher_name = "/response_text";
    declare_parameter<std::string>("gpt_topic_pub", topic_publisher_name);
    get_parameter<std::string>("gpt_topic_pub", topic_publisher_name);
    publisher_ = create_publisher<std_msgs::msg::String>(topic_publisher_name,
                                                         rclcpp::QoS(10));

    thread_exit_ = false;
    thread_ = std::thread(&GPTNode::Run, this);
  }

  ~GPTNode() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      thread_exit_ = true;
    }
    cv_.notify_all();

    thread_.join();
  }

private:
  GPTConfig ParseConfigFile(const std::string &filename) {
    GPTConfig config;
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
      std::cerr << "Failed to open config file." << std::endl;
      throw std::runtime_error("Failed to open config file.");
    }

    nlohmann::ordered_json json_data;
    input_file >> json_data;

    config.api_key = json_data["api_key"];
    config.chat_mode_enable = json_data["chat_mode_enable"];
    config.model = json_data["model"];
    config.temperature = json_data["temperature"];
    config.max_tokens = json_data["max_tokens"];
    config.top_p = json_data["top_p"];
    config.frequency_penalty = json_data["frequency_penalty"];
    config.presence_penalty = json_data["presence_penalty"];

    input_file.close();
    return config;
  }

  void InputCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message_queue_.size() >= kMaxMessageQueueSize) {
      message_queue_.pop();
    }
    message_queue_.push(msg);
    cv_.notify_one();
  }

  void ProcessMessage(const std_msgs::msg::String::SharedPtr msg,
                      GPTConfig config) {
    std::string input = msg->data;
    std::cout << "user: " << input << std::endl;

    nlohmann::ordered_json content_user;
    content_user["role"] = "user";
    content_user["content"] = input;
    context_.push_back(content_user);

    std::string reply = SendChatGPTRequest(context_, config);
    std::cout << "assistant: " << reply << std::endl;

    if (config.chat_mode_enable) {
      nlohmann::ordered_json content_assistant;
      content_assistant["role"] = "assistant";
      content_assistant["content"] = reply;
      context_.push_back(content_assistant);
    }

    auto output_msg = std_msgs::msg::String();
    output_msg.data = reply;
    publisher_->publish(output_msg);
  }

  void Run(void) {
    GPTConfig gpt_config = ParseConfigFile("config/gpt_config.json");

    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock,
               [this] { return !message_queue_.empty() || thread_exit_; });

      if (thread_exit_) {
        break;
      }

      auto message = message_queue_.front();
      message_queue_.pop();
      lock.unlock();

      ProcessMessage(message, gpt_config);
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  nlohmann::ordered_json context_;

  static constexpr size_t kMaxMessageQueueSize = 10;
  bool thread_exit_;
  std::queue<std_msgs::msg::String::SharedPtr> message_queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
};

} // namespace gpt_node

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpt_node::GPTNode>());
  rclcpp::shutdown();
  return 0;
}
