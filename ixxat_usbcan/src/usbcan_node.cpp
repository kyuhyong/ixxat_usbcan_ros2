#include <chrono>
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ixxat_interfaces/msg/can_message.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class UsbCanNode : public rclcpp::Node
{
	public:
		UsbCanNode()
		: Node("UsbCan_Node")
		{
			if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
				perror("Socket");
				return;
			}
			strcpy(ifr.ifr_name, "can0");
			ioctl(s, SIOCGIFINDEX, &ifr);

			memset(&addr, 0, sizeof(addr));
			addr.can_family = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;
			if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				perror("Bind");
				return;
			}
			sub_canMessage_ = this->create_subscription<ixxat_interfaces::msg::CanMessage>(
				"can_TxMsg", 10, std::bind(&UsbCanNode::cb_newCanMessage, this, _1));
			pub_canMessage_ = this->create_publisher<ixxat_interfaces::msg::CanMessage>(
				"can_RxMsg", 10);
			timer_ = this->create_wall_timer(
        		20ns, std::bind(&UsbCanNode::cb_timer, this));
		}
	private:
		int s;
		struct sockaddr_can addr;
		struct ifreq ifr;
		rclcpp::Publisher<ixxat_interfaces::msg::CanMessage>::SharedPtr pub_canMessage_;
		rclcpp::TimerBase::SharedPtr timer_;
		void cb_timer()
		{
			struct can_frame rxframe;
			int nbytes = recv(s, &rxframe, sizeof(struct can_frame), MSG_DONTWAIT);
			if(nbytes > 0) {
				ixxat_interfaces::msg::CanMessage canMsg = ixxat_interfaces::msg::CanMessage();
				canMsg.can_id = rxframe.can_id;
				canMsg.can_dlc = rxframe.can_dlc;
				
				printf("0x%03X [%d] ",rxframe.can_id, rxframe.can_dlc);
				for (int i = 0; i < rxframe.can_dlc; i++) {
					printf("%02X ",rxframe.data[i]);
					canMsg.data_array[i] = rxframe.data[i];
				}
				printf("\r\n");
				pub_canMessage_->publish(canMsg);

			}
		}
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
