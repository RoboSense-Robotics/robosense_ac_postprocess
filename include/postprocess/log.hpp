/************************************************************************
Copyright 2025 RoboSense Technology Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
************************************************************************/
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <ctime>
#include <sstream>

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLogLevel(LogLevel level) {
        logLevel_ = level;
    }

    void enableConsoleOutput(bool enable) {
        consoleOutputEnabled_ = enable;
    }

    void enableFileOutput(const std::string& filename) {
        fileStream_.open(filename, std::ios::app);
        if (!fileStream_.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
        }
    }

    void disableFileOutput() {
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
    }

    Logger& operator<<(const std::string& message) {
        log(currentLevel_, message);
        return *this;
    }

    template<typename T>
    Logger& operator<<(T value) {
        stream_ << value;
        // log(currentLevel_, stream_.str());
        return *this;
    }

    Logger& operator<<(LogLevel level) {
        currentLevel_ = level;
        return *this;
    }

private:
    Logger() : logLevel_(LogLevel::INFO), consoleOutputEnabled_(true), currentLevel_(LogLevel::DEBUG) {}

    ~Logger() {
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(LogLevel level, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (level >= logLevel_) {
            std::string logMessage = formatLogMessage(level, message);
            if (consoleOutputEnabled_) {
                std::cout << logMessage;
            }
            if (fileStream_.is_open()) {
                fileStream_ << logMessage;
            }
        }
        // Clear the stringstream after logging
        stream_.str("");
        stream_.clear();
    }

    std::string formatLogMessage(LogLevel level, const std::string& message) {
        time_t now = time(nullptr);
        char* dt = ctime(&now);
        dt[strlen(dt) - 1] = '\0'; // Remove newline character

        std::string levelStr;
        switch (level) {
            case LogLevel::DEBUG: levelStr = "DEBUG"; break;
            case LogLevel::INFO: levelStr = "INFO"; break;
            case LogLevel::WARN: levelStr = "WARN"; break;
            case LogLevel::ERROR: levelStr = "ERROR"; break;
            case LogLevel::FATAL: levelStr = "FATAL"; break;
        }

        return "[" + std::string(dt) + "] [" + levelStr + "] " + message;
    }

    LogLevel logLevel_;
    bool consoleOutputEnabled_{true};
    std::ofstream fileStream_;
    std::mutex mutex_;
    std::stringstream stream_;
    LogLevel currentLevel_;
};

#define LOG Logger::getInstance()
#define RDEBUG LOG << "\n" << LogLevel::DEBUG
#define RINFO LOG << "\n" << LogLevel::INFO
#define RWARN LOG << "\n" << LogLevel::WARN
#define RERROR LOG << "\n" << LogLevel::ERROR
#define RFATAL LOG << "\n" << LogLevel::FATAL
// int main() {
//     // 配置日志系统
//     LOG.setLogLevel(LogLevel::DEBUG);
//     LOG.enableConsoleOutput(true);
//     LOG.enableFileOutput("application.log");

//     // 记录不同级别的日志
//     LOG << LogLevel::DEBUG << "This is a debug message" << std::endl;
//     LOG << LogLevel::INFO << "This is an info message" << std::endl;
//     LOG << LogLevel::WARN << "This is a warning message" << std::endl;
//     LOG << LogLevel::ERROR << "This is an error message" << std::endl;
//     LOG << LogLevel::FATAL << "This is a fatal message" << std::endl;

//     // 使用流操作符记录复杂消息
//     int search_idx = 10;
//     LOG << LogLevel::WARN << "Motion Correct Warning, findFrames search_idx out of range: " << search_idx << std::endl;

//     return 0;
// }



