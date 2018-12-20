/**
 * redis_client.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 1, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_UTILS_REDIS_CLIENT_H_
#define SPATIAL_DYN_UTILS_REDIS_CLIENT_H_

#include <exception>   // std::exception
#include <future>      // std::future, std::promise
#include <functional>  // std::function
#include <string>      // std::string
#include <sstream>     // std::stringstream

#include <cpp_redis/cpp_redis>

namespace SpatialDyn {

class RedisClient : public cpp_redis::client {

 public:

  RedisClient() : cpp_redis::client() {}

  template<typename T>
  RedisClient& set(const std::string& key, const T& value,
                   const reply_callback_t& reply_callback);
  template<typename T>
  RedisClient& publish(const std::string& key, const T& value,
                       const reply_callback_t& reply_callback);

  template<typename T>
  std::future<cpp_redis::reply> set(const std::string& key, const T& value);
  template<typename T>
  std::future<cpp_redis::reply> publish(const std::string& key, const T& value);

  template<typename T>
  cpp_redis::reply sync_set(const std::string& key, const T& value);
  template<typename T>
  cpp_redis::reply sync_publish(const std::string& key, const T& value);

  template<typename T>
  RedisClient& get(const std::string& key,
                   const std::function<void(T)>& reply_callback,
                   const std::function<void(const std::string&)>& error_callback =
                       std::function<void(const std::string&)>());

  template<typename T>
  std::future<T> get(const std::string& key);

  template<typename T>
  T sync_get(const std::string& key);

};


template<typename T>
RedisClient& RedisClient::set(const std::string& key, const T& value,
                              const reply_callback_t& reply_callback) {
  std::stringstream ss;
  ss << value;
  cpp_redis::client::set(key, ss.str(), reply_callback);
  return *this;
}
template<typename T>
RedisClient& RedisClient::publish(const std::string& key, const T& value,
                                  const reply_callback_t& reply_callback) {
  cpp_redis::client::publish(key, std::string(value), reply_callback);
  return *this;
}

template<typename T>
std::future<cpp_redis::reply> RedisClient::set(const std::string& key, const T& value) {
  auto promise = std::make_shared<std::promise<cpp_redis::reply>>();
  set(key, value, [promise](cpp_redis::reply& reply) {
    promise->set_value(reply);
  });
  return promise->get_future();
}
template<typename T>
std::future<cpp_redis::reply> RedisClient::publish(const std::string& key, const T& value) {
  auto promise = std::make_shared<std::promise<cpp_redis::reply>>();
  publish(key, value, [promise](cpp_redis::reply& reply) {
    promise->set_value(reply);
  });
  return promise->get_future();
}

template<typename T>
cpp_redis::reply RedisClient::sync_set(const std::string& key, const T& value) {
  std::future<cpp_redis::reply> future = set(key, value);
  commit();
  return future.get();
};
template<typename T>
cpp_redis::reply RedisClient::sync_publish(const std::string& key, const T& value) {
  std::future<cpp_redis::reply> future = publish(key, value);
  commit();
  return future.get();
};

template<typename T>
RedisClient& RedisClient::get(const std::string& key,
    const std::function<void(T)>& reply_callback,
    const std::function<void(const std::string&)>& error_callback) {
  cpp_redis::client::get(key, [key, reply_callback, error_callback](cpp_redis::reply& reply) {
    if (!reply.is_string()) {
      if (error_callback) {
        error_callback("RedisClient::get(): Failed to get string value from key: " + key + ".");
      }
      return;
    }
    std::stringstream ss(reply.as_string());
    T value;
    try {
      ss >> value;
    } catch (const std::exception& e) {
      if (error_callback) {
        error_callback("RedisClient::get(): Exception thrown on key: " + key + "\n\t" + e.what());
      }
      return;
    }
    reply_callback(std::move(value));
  });
  return *this;
}

template<typename T>
std::future<T> RedisClient::get(const std::string& key) {
  auto promise = std::make_shared<std::promise<T>>();
  get<T>(key, [key, promise](T val) {
    promise->set_value(std::move(val));
  }, [promise](const std::string& error) {
    promise->set_exception(std::make_exception_ptr(std::runtime_error(error)));
  });
  return promise->get_future();
}

template<typename T>
T RedisClient::sync_get(const std::string& key) {
  std::future<T> future = get<T>(key);
  commit();
  return future.get();
};

}  // namespace dex

#endif  // SPATIAL_DYN_UTILS_REDIS_CLIENT_H_
