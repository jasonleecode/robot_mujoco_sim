#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>

class Timer {
public:
    Timer() : _running(false) {}

    ~Timer() {
        stop(); // 析构时确保线程停止，防止 core dump
    }

    // 开始定时任务
    // interval: 间隔时间 (毫秒)
    // task: 要执行的函数
    void start(int interval_ms, std::function<void()> task) {
        // 如果已经在运行，先停止
        if (_running) stop();

        _running = true;
        _thread = std::thread([this, interval_ms, task]() {
            while (_running) {
                std::unique_lock<std::mutex> lock(_mutex);
                
                // 核心逻辑：wait_for
                // 它会等待 interval_ms 时间，或者直到 stop() 被调用（_running 变 false）
                // 如果是因为超时（时间到了）返回，result 为 false -> 执行任务
                // 如果是因为 stop 被调用返回，result 为 true -> 退出循环
                auto result = _cv.wait_for(lock, std::chrono::milliseconds(interval_ms), [this]{ 
                    return !_running; 
                });

                if (result) {
                    // _running 变成了 false，说明被要求停止
                    break; 
                }
                
                // 时间到了，执行任务
                task();
            }
        });
    }

    // 停止定时任务
    void stop() {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _running = false;
        }
        _cv.notify_one(); // 唤醒沉睡的线程，让它立刻退出

        if (_thread.joinable()) {
            _thread.join(); // 等待线程安全结束
        }
    }

    bool isRunning() const {
        return _running;
    }

private:
    std::atomic<bool> _running;
    std::thread _thread;
    std::mutex _mutex;
    std::condition_variable _cv;
};