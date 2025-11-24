#pragma once

#include <functional>
#include <cstdint>
#include <forward_list>

namespace AdamLib
{


struct SignalGeneric
{
  SignalGeneric();
  virtual ~SignalGeneric();
  virtual void disconnect(uint32_t _id) = 0;

  SignalGeneric(const SignalGeneric&) = delete;
  SignalGeneric(SignalGeneric&&) = delete;
  void operator=(const SignalGeneric&) = delete;
  void operator=(SignalGeneric&&) = delete;

  uint32_t signal_id_;
};

struct ConnectionController
{
  ConnectionController(SignalGeneric* _signal, uint32_t _connection_id);
  ConnectionController(ConnectionController&& _c) noexcept ;
  ~ConnectionController();

  void operator=(ConnectionController&& _c);

  //multiple controllers with the same connection_id is forbidden
  ConnectionController(const ConnectionController&) = delete;
  void operator=(const ConnectionController&) = delete;


  void disconnect();

private:
  SignalGeneric* signal_ = nullptr;
  uint32_t signal_id_{0}; 
  uint32_t connection_id_{0};

  void reset();

};



//! Signal Class
/*!
    Contains list of callbacks that will be called upon signal emission.
    Arguments provided upon emission will be passed to all callbacks as parameters.

    Connecting to a signal will provide user with ConnectionController object, allowing them to disconnect at will.
    ConnectionController will also disconnect upon destruction.

*/
template<typename... Arguments>
class Signal final : public SignalGeneric
{
  struct f
  {
    uint32_t connection_id_{0};
    std::function<void(Arguments...)> func;
    f(const std::function<void(Arguments...)> _func, uint32_t _connection_id) : func(_func), connection_id_(_connection_id) {}
  };

  uint32_t next_id{0};

  std::forward_list<f> callbacks;


public:

  Signal() = default;
  Signal(const Signal&) = delete;
  Signal(Signal&&) = delete;
  void operator=(const Signal&) = delete;
  void operator=(Signal&&) = delete;

  [[nodiscard]] inline ConnectionController connect(std::function<void(Arguments...)> _func)
  {
    callbacks.emplace_front(_func, ++next_id);
    return ConnectionController(this, next_id);
  }

  inline void emit(Arguments... args)
  {
    for(auto& f : callbacks)
    {
      f.func(args...);
    }
  }
  
  inline void disconnect(uint32_t _connection_id)
  {
    callbacks.remove_if([_connection_id](const f& func){return func.connection_id_ == _connection_id;});
  }

  inline void clear()
  {
    callbacks.clear();
  }


  inline void operator()(Arguments... args) {emit(args...);}

};


  #define RegisterCustomConnection(Signal, Method) registerConnection(Signal.connect([this](auto ...args){dodick(args...);}));

}