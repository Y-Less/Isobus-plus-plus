//================================================================================================
/// @file socket_can_interface.cpp
///
/// @brief An CAN driver for socket CAN on linux.
/// @author Adrian Del Grosso
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/isobus/can_NAME.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>

SocketCANInterface::SocketCANInterface(const std::string deviceName) :
  pCANDevice(new sockaddr_can),
  name(deviceName),
  fileDescriptor(-1)
{
	if (nullptr != pCANDevice)
	{
		std::memset(pCANDevice, 0, sizeof(struct sockaddr_can));
	}
}

SocketCANInterface::~SocketCANInterface()
{
	close();

	if (nullptr != pCANDevice)
	{
		delete pCANDevice;
		pCANDevice = nullptr;
	}
}

bool SocketCANInterface::get_is_valid() const
{
	return (-1 != fileDescriptor);
}

std::string SocketCANInterface::get_device_name() const
{
	return name;
}

void SocketCANInterface::close()
{
	::close(fileDescriptor);
	fileDescriptor = -1;
}

void SocketCANInterface::open()
{
	fileDescriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fileDescriptor >= 0)
	{
		struct ifreq interfaceRequestStructure;
		const int RECEIVE_OWN_MESSAGES = 0;
		const int DROP_MONITOR = 1;
		const int TIMESTAMPING = 0x58;
		const int TIMESTAMP = 1;
		memset(&interfaceRequestStructure, 0, sizeof(interfaceRequestStructure));
		strncpy(interfaceRequestStructure.ifr_name, name.c_str(), sizeof(interfaceRequestStructure.ifr_name));
		setsockopt(fileDescriptor, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &RECEIVE_OWN_MESSAGES, sizeof(RECEIVE_OWN_MESSAGES));
		setsockopt(fileDescriptor, SOL_SOCKET, SO_RXQ_OVFL, &DROP_MONITOR, sizeof(DROP_MONITOR));

		if (setsockopt(fileDescriptor, SOL_SOCKET, SO_TIMESTAMPING, &TIMESTAMPING, sizeof(TIMESTAMPING)) < 0)
		{
			setsockopt(fileDescriptor, SOL_SOCKET, SO_TIMESTAMP, &TIMESTAMP, sizeof(TIMESTAMP));
		}

		if (ioctl(fileDescriptor, SIOCGIFINDEX, &interfaceRequestStructure) >= 0)
		{
			memset(pCANDevice, 0, sizeof(sockaddr_can));
			pCANDevice->can_family = AF_CAN;
			pCANDevice->can_ifindex = interfaceRequestStructure.ifr_ifindex;

			if (bind(fileDescriptor, (struct sockaddr *)pCANDevice, sizeof(struct sockaddr)) < 0)
			{
				close();
			}
		}
		else
		{
			close();
		}
	}
	else
	{
		close();
	}
}

bool SocketCANInterface::read_frame(isobus::HardwareInterfaceCANFrame &canFrame)
{
	struct pollfd pollingFileDescriptor;
	bool retVal = false;

	pollingFileDescriptor.fd = fileDescriptor;
	pollingFileDescriptor.events = POLLIN;
	pollingFileDescriptor.revents = 0;

	if (1 == poll(&pollingFileDescriptor, 1, 100))
	{
		canFrame.timestamp_us = std::numeric_limits<std::uint64_t>::max();
		struct can_frame txFrame;
		struct msghdr message;
		struct iovec segment;

		char lControlMessage[CMSG_SPACE(sizeof(struct timeval) + (3 * sizeof(struct timespec)) + sizeof(std::uint32_t))];

		segment.iov_base = &txFrame;
		segment.iov_len = sizeof(struct can_frame);
		message.msg_iov = &segment;
		message.msg_iovlen = 1;
		message.msg_control = &lControlMessage;
		message.msg_controllen = sizeof(lControlMessage);
		message.msg_name = pCANDevice;
		message.msg_namelen = sizeof(struct sockaddr_can);
		message.msg_flags = 0;

		if (recvmsg(fileDescriptor, &message, 0) > 0)
		{
			if (0 == (txFrame.can_id & CAN_ERR_FLAG))
			{
				bool printtt = false;
				if (0 != (txFrame.can_id & CAN_EFF_FLAG))
				{
					canFrame.identifier = (txFrame.can_id & CAN_EFF_MASK);
					canFrame.isExtendedFrame = true;
					uint32_t id = canFrame.identifier;
					uint32_t pgn = ((0x00F00000 & id) < 0x00F00000) ? ((id >> 8) & 0x0003FF00) : ((id >> 8) & 0x0003FFFF);
					uint32_t src = ((0x00F00000 & id) < 0x00F00000) ? (id & 0xFF) : 0xFF;
					uint32_t dst = ((0x00F00000 & id) < 0x00F00000) ? ((id >> 8) & 0xFF) : 0xFF;
					if (printtt) printf("LONG: 0x%08x\n\t- PGN: 0x%04x\n\t- SRC: 0x%02x\n\t- DST: 0x%02x\n", id, pgn, src, dst);
				}
				else
				{
					canFrame.identifier = (txFrame.can_id & CAN_SFF_MASK);
					canFrame.isExtendedFrame = false;
					uint32_t id = canFrame.identifier;
					printtt = id == 0x0283 || id == 0x020b;
					printtt = false;
					if (printtt) printf("SHORT: 0x%04x\n", id);
				}
				canFrame.dataLength = txFrame.can_dlc;
				memset(canFrame.data, 0, sizeof(canFrame.data));
				memcpy(canFrame.data, txFrame.data, canFrame.dataLength);

				if (printtt) switch (canFrame.dataLength)
				{
				case 0:
					printf("\t- DAT: \n");
					break;
				case 1:
					printf("\t- DAT: %02x\n", canFrame.data[0]);
					break;
				case 2:
					printf("\t- DAT: %02x %02x\n", canFrame.data[0], canFrame.data[1]);
					break;
				case 3:
					printf("\t- DAT: %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2]);
					break;
				case 4:
					printf("\t- DAT: %02x %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2], canFrame.data[3]);
					break;
				case 5:
					printf("\t- DAT: %02x %02x %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2], canFrame.data[3], canFrame.data[4]);
					break;
				case 6:
					printf("\t- DAT: %02x %02x %02x %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2], canFrame.data[3], canFrame.data[4], canFrame.data[5]);
					break;
				case 7:
					printf("\t- DAT: %02x %02x %02x %02x %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2], canFrame.data[3], canFrame.data[4], canFrame.data[5], canFrame.data[6]);
					break;
				case 8:
					printf("\t- DAT: %02x %02x %02x %02x %02x %02x %02x %02x\n", canFrame.data[0], canFrame.data[1], canFrame.data[2], canFrame.data[3], canFrame.data[4], canFrame.data[5], canFrame.data[6], canFrame.data[7]);
					{
						std::uint64_t claimedNAME;
						claimedNAME = canFrame.data[0];
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[1]) << 8);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[2]) << 16);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[3]) << 24);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[4]) << 32);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[5]) << 40);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[6]) << 48);
						claimedNAME |= (static_cast<std::uint64_t>(canFrame.data[7]) << 56);
						isobus::NAME nn(claimedNAME);
						printf("\t- NAM:\n\n\t\t\tarbitrary_address_capable: %d\n\t\t\tindustry_group           : %02x\n\t\t\tdevice_class_instance    : %02x\n\t\t\tdevice_class             : %02x\n\t\t\tfunction_code            : %02x\n\t\t\tfunction_instance        : %02x\n\t\t\tecu_instance             : %02x\n\t\t\tmanufacturer_code        : %04x\n\t\t\tidentity_number          : %08x\n\n", nn.get_arbitrary_address_capable(), nn.get_industry_group(), nn.get_device_class_instance(), nn.get_device_class(), nn.get_function_code(), nn.get_function_instance(), nn.get_ecu_instance(), nn.get_manufacturer_code(), nn.get_identity_number());
					}
					break;
				}

				for (struct cmsghdr *pControlMessage = CMSG_FIRSTHDR(&message); (nullptr != pControlMessage) && (SOL_SOCKET == pControlMessage->cmsg_level); pControlMessage = CMSG_NXTHDR(&message, pControlMessage))
				{
					switch (pControlMessage->cmsg_type)
					{
						case SO_TIMESTAMP:
						{
							struct timeval *time = (struct timeval *)CMSG_DATA(pControlMessage);

							if (std::numeric_limits<std::uint64_t>::max() == canFrame.timestamp_us)
							{
								canFrame.timestamp_us = static_cast<std::uint64_t>(time->tv_usec) + (static_cast<std::uint64_t>(time->tv_sec) * 1000000);
							}
						}
						break;

						case SO_TIMESTAMPING:
						{
							struct timespec *time = (struct timespec *)(CMSG_DATA(pControlMessage));
							canFrame.timestamp_us = (static_cast<std::uint64_t>(time[2].tv_nsec) / 1000) + (static_cast<std::uint64_t>(time[2].tv_sec) * 1000000);
						}
						break;
					}
				}
				retVal = true;
			}
		}
		else if (errno == ENETDOWN)
		{
			isobus::CANStackLogger::CAN_stack_log(isobus::CANStackLogger::LoggingLevel::Critical, "[SocketCAN] " + get_device_name() + " interface is down.");
			close();
		}
	}
	else if (pollingFileDescriptor.revents & (POLLERR | POLLHUP))
	{
		close();
	}
	return retVal;
}

bool SocketCANInterface::write_frame(const isobus::HardwareInterfaceCANFrame &canFrame)
{
	struct can_frame txFrame;
	bool retVal = false;

	txFrame.can_id = canFrame.identifier;
	txFrame.can_dlc = canFrame.dataLength;
	memcpy(txFrame.data, canFrame.data, canFrame.dataLength);

	if (canFrame.isExtendedFrame)
	{
		txFrame.can_id |= CAN_EFF_FLAG;
	}

	if (write(fileDescriptor, &txFrame, sizeof(struct can_frame)) > 0)
	{
		retVal = true;
	}
	else if (errno == ENETDOWN)
	{
		isobus::CANStackLogger::CAN_stack_log(isobus::CANStackLogger::LoggingLevel::Critical, "[SocketCAN] " + get_device_name() + " interface is down.");
		close();
	}
	return retVal;
}
