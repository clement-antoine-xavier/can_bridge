#include "../include/tcp_controller.h"

tcp_controller::tcp_controller(const char *host, uint16_t port)
    : _host(host), _port(port)
{
}

bool tcp_controller::connect()
{
  if (_connected)
  {
    return true;
  }

  // Non-blocking dial; poll until connected or timed out. WiFiClient::connect
  // on WiFiNINA returns WL_CONNECTED on success.
  int result = _client.connect(_host, _port);
  if (result == WL_CONNECTED)
  {
    _connected = true;
    return true;
  }

  // The connect() above already returned, so give the stack a short window
  // for the connection to settle on the non-blocking clients.
  uint32_t start = millis();
  while ((millis() - start) < CONNECT_TIMEOUT_MS)
  {
    if (_client.connected())
    {
      _connected = true;
      return true;
    }
    if (!_client.connected() && !_client.available())
    {
      // Underlying socket closed while waiting.
      break;
    }
    delay(10);
  }

  _client.stop();
  _connected = false;
  return false;
}

bool tcp_controller::send(const uint8_t *data, size_t len)
{
  if (!isConnected())
  {
    return false;
  }

  size_t i = 0;
  while (i < len)
  {
    size_t written = _client.write(data + i, len - i);
    if (written == 0)
    {
      // The socket may have closed under us.
      _connected = _client.connected();
      return false;
    }
    i += written;
  }
  return true;
}

bool tcp_controller::recv(uint8_t *out_data, size_t &out_len, uint32_t timeout_ms)
{
  if (!isConnected())
  {
    return false;
  }

  size_t capacity = out_len;
  out_len = 0;

  uint32_t start = millis();
  while (out_len == 0 && (millis() - start) < timeout_ms)
  {
    while (_client.available() && out_len < capacity)
    {
      out_data[out_len++] = (uint8_t)_client.read();
    }
    if (out_len > 0)
    {
      return true;
    }
    delay(RECV_CHUNK_MS);
  }

  // If the peer closed the connection, mark it disconnected.
  if (!_client.connected())
  {
    _connected = false;
  }
  return out_len > 0;
}

bool tcp_controller::isConnected() const
{
  // _connected tracks our intent; _client.connected() reflects the real state.
  return _connected && _client.connected();
}

void tcp_controller::disconnect()
{
  _client.stop();
  _connected = false;
}

const char *tcp_controller::status() const
{
  return isConnected() ? "connected" : "disconnected";
}
