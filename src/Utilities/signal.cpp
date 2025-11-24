#include <AdamLib/Utilities/Signal.hpp>
#include <AdamLib/Defines.hpp>

#include <atomic>

using namespace AdamLib;


std::atomic<uint32_t> id_gen{0};
bool signals[MAXIMUM_SIGNALS];

uint32_t getUniqueSignalID()
{
  uint32_t n_id = ++id_gen;
  signals[n_id] = true;
  return n_id;
}

void removeSignal(uint32_t _signal_id)
{
  signals[_signal_id] = false;
}

bool isSignalAlive(uint32_t _signal_id)
{
  return signals[_signal_id];
}

//-----

SignalGeneric::SignalGeneric() : signal_id_(getUniqueSignalID()){}

SignalGeneric::~SignalGeneric(){removeSignal(signal_id_);}

//-----

ConnectionController::ConnectionController(SignalGeneric* _signal, uint32_t _connection_id) 
: signal_(_signal), connection_id_(_connection_id), signal_id_(_signal->signal_id_) {}

ConnectionController::ConnectionController(ConnectionController&& _c) noexcept : signal_(_c.signal_), connection_id_(_c.connection_id_), signal_id_(_c.signal_id_)
{
  _c.reset();
}

ConnectionController::~ConnectionController()
{
  disconnect();
}

void ConnectionController::operator=(ConnectionController&& _c)
{
  disconnect();
  signal_ = _c.signal_;
  connection_id_ = _c.connection_id_;
  signal_id_ = _c.signal_id_;

  _c.reset();
}

void ConnectionController::disconnect()
{
  if(signal_ && isSignalAlive(signal_id_))
  {
    signal_->disconnect(connection_id_);
    reset();
  }
}

void ConnectionController::reset()
{
  signal_ = nullptr;
  connection_id_ = 0;
  signal_id_ = 0;
}
