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

