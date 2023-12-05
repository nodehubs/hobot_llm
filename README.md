# GPTNode

**gpt_node**提供和ChatGPT交互功能，订阅文本消息，调用ChatGPT API返回结果，然后再将结果发送出去。当前支持两种交互模式，一种是聊天模式，该模式将历史对话叠加送给ChatGPT以实现多轮对话，该方式消耗token数较多，另一种方式是问答模式，该模式只支持单轮对话，即用户提问给ChatGPT，然后返回答案，下一次又是一轮全新的对话，交互模式可通过 *gpt_config.json* 配置文件中的 `chat_mode_enable` 字段配置，默认模式为问答模式。

## 运行方式

1. 拷贝配置文件到当前目录

   ```bash
   cp -rf /opt/tros/lib/gpt_node/config ./
   ```

2. 修改 *config/gpt_config.json* ，将**api_key**字段设置为自己的ChatGPT API Key

3. 确认网络可以访问ChatGPT

4. 运行Node

   ```bash
   source /opt/tros/setup.bash

   ros2 run gpt_node gpt_node
   ```

   程序运行成功后，默认订阅topic "/request_text"（std_msgs/msg/String类型），将结果以topic "/response_text"（std_msgs/msg/String类型）发布出去。

   可使用如下命令发送消息验证程序是否运行成功：

   ```bash
   source /opt/tros/setup.bash
   ros2 topic pub --once /request_text std_msgs/msg/String "{data: "你是谁"}"
   ```

   注意：普通用户ChatGPT API Key有调用频率限制，一般限制为每分钟三次。

## 参数列表

| 参数名          | 解释            | 类型        | 是否必须 | 默认值      |
| --------------- | -------------- | ----------- | --------| ----------- |
| gpt_topic_sub   | 订阅的文本topic | std::string | 否      | "/request_text" |
| gpt_topic_pub   | 发布的文本topic | std::string | 否      | "/response_text" |
