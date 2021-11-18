//
// Created by rnb on 21. 10. 2..
//
#include "logger.h"

std::string RNB::WRAP_LOG_FRAME(const char* msg){
    int msg_len = strlen(msg);
    float margin = fmax(2, LOG_FRAME_LEN-msg_len);
    std::string prefix(floor(margin/2)-1, '=');
    std::string postfix(ceil(margin/2)-1, '=');
    std::string msg_line = prefix + " " + msg + " " + postfix + "\n";
    return msg_line;
}

std::string RNB::WRAP_LOG_FRAME(std::string& msg) {
    return WRAP_LOG_FRAME(msg.c_str());
}

void RNB::PRINT_ERROR(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg){
    std::string msg_line = "\x1B[31m" + WRAP_LOG_FRAME(msg) + "\033[0m";
    std::cout<<TEXT_RED(LOG_FRAME_LINE);
    std::cout<<msg_line.c_str();
    std::cout<<TEXT_RED(LOG_FRAME_LINE "\n");
}

void RNB::PRINT_ERROR(std::string& msg) {
    PRINT_ERROR(msg.c_str());
}

void RNB::PRINT_FRAMED_LOG(std::basic_string<char, std::char_traits<char>, std::allocator<char>> msg, bool endl){
    std::cout<<WRAP_LOG_FRAME(msg);
    if(endl) std::cout<<std::endl;
}

void RNB::PRINT_FRAMED_LOG(std::string& msg, bool endl){
    PRINT_FRAMED_LOG(msg.c_str(), endl);
}