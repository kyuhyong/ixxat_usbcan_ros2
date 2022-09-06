#include <chrono>
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ixxat_interfaces/msg/can_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class UsbCanTalker : public rclcpp::Node
{
	public:
		UsbCanTalker()
		: Node("UsbCan_Talker")
		{
			pub_canMessage_ = this->create_publisher<ixxat_interfaces::msg::CanMessage>(
				"can_TxMsg", 10);
      		timer_ = this->create_wall_timer(
        		500ms, std::bind(&UsbCanTalker::timer_callback, this));
		}
	private:
		rclcpp::Publisher<ixxat_interfaces::msg::CanMessage>::SharedPtr pub_canMessage_;
		void timer_callback()
		{
			auto canMsg = ixxat_interfaces::msg::CanMessage();
			canMsg.can_id = 55;
			canMsg.can_dlc = 4;
			for(int i = 0 ; i<4; i++) {
				canMsg.data_array[i] = 0;
			}
			RCLCPP_INFO(this->get_logger(), "Publishing: ");
			pub_canMessage_->publish(canMsg);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		void cb_newCanMessage(const ixxat_interfaces::msg::CanMessage::SharedPtr msg) const
		{
			RCLCPP_INFO(this->get_logger(), "Send CAN ID:%d, DLC:%d", msg->can_id, msg->can_dlc);
			int length = static_cast<int>(msg->can_dlc);
			for(int i =0; i < length; i++) {
				printf("%d ", msg->data_array[i]);
			}
			printf("\r\n");
		}
		rclcpp::Subscription<ixxat_interfaces::msg::CanMessage>::SharedPtr sub_canMessage_;
};

int main(int argc, char * argv[])
{
  	printf("hello world ixxat_tx package\n");
  	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UsbCanTalker>());
	rclcpp::shutdown();
  return 0;
}
