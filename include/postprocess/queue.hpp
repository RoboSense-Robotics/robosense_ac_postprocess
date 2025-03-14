/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef HYPER_VISION_POSTPROCESS_SYNCQUEUE_H_
#define HYPER_VISION_POSTPROCESS_SYNCQUEUE_H_

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SyncVector {
public:
  SyncVector() = default;
  ~SyncVector() = default;

  void add(const T &element) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.size() >5) {
      data_.pop_front();
    }
    data_.push_back(element);
  }

  T front() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.empty())
    {
      throw std::out_of_range("Vector is empty");
    }
    return data_.front();
  }

  T back() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.empty())
    {
      throw std::out_of_range("Vector is empty");
    }
    return data_.back();
  }


  bool remove(const T &element) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find(data_.begin(), data_.end(), element);
    if (it != data_.end())
    {
      data_.erase(it);
      return true;
    }
    return false;
  }

  T get(size_t index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < data_.size())
    {
      return data_[index];
    }
    throw std::out_of_range("Index out of range");
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.size();
  }

private:
  std::deque<T> data_;
  mutable std::mutex mutex_;
};

template <typename T>
class SyncQueue {
public:
  SyncQueue() {
    max_size_ = 30;
  }
  SyncQueue(size_t max_size) : max_size_(max_size) {}
  ~SyncQueue() {}

  void push(const T& value) {
    std::unique_lock<std::mutex> lock(mtx_);
    queue_.push_back(value);
    if (queue_.size() > max_size_) {
      queue_.pop_front();
    }
    cv_.notify_one();
  }

  T pop() {
    T value;
    std::unique_lock<std::mutex> lock(mtx_);
    // cv_.wait_for(lock, std::chrono::microseconds(10000), [this] { return (!queue_.empty()); });
    cv_.wait(lock, [this] { return !queue_.empty(); });
    if (!queue_.empty()) {
      value = queue_.front();
      queue_.pop_front();
    }
    return value;
  }

  // inline void clear()
  // {
  //   std::queue<T> empty;
  //   std::lock_guard<std::mutex> lg(mtx_);
  //   swap(empty, queue_);
  // }

  bool empty() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return queue_.empty();
  }

  inline size_t size() {
    std::unique_lock<std::mutex> lock(mtx_);
    return queue_.size();
  }

private:
  std::deque<T> queue_;
  size_t max_size_;
  std::mutex mtx_;
  std::condition_variable cv_;
};


#endif // HYPER_VISION_POSTPROCESS_SYNCQUEUE_H_

