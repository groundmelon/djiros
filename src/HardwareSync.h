#pragma once

#include <std_msgs/Header.h>
#include <mutex>

struct SyncSession_t {
	std::mutex locker;
    enum class Status { Free, RecvReq, ForwardReq, RecvAck, ProcessAck };
    Status status;
    int freq;
    std_msgs::Header header;

    SyncSession_t() : status(Status::Free), freq(0){};
};
