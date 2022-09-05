#include <chrono>
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ixxat_interfaces/msg/can_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class UsbCanNode : public rclcpp::Node
{
	public:
		UsbCanNode()
		: Node("UsbCan_Node")
		{
			sub_canMessage_ = this->create_subscription<ixxat_interfaces::msg::CanMessage>(
				"can_TxMsg", 10, std::bind(&UsbCanNode::cb_newCanMessage, this, _1));
			pub_canMessage_ = this->create_publisher<ixxat_interfaces::msg::CanMessage>(
				"can_RxMsg", 10);

		}
	private:
		rclcpp::Publisher<ixxat_interfaces::msg::CanMessage>::SharedPtr pub_canMessage_;
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
	printf("hello world ixxat_usbcan package\n");
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UsbCanNode>());
	rclcpp::shutdown();
	return 0;
}
