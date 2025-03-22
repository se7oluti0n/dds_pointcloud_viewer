#pragma once

#include <dds/dds.hpp>
#include <fstream>

using namespace org::eclipse::cyclonedds::core;
template <typename T>
void WriteSomeMessagesToFile(const T &sample, const std::string &filename) {
  // set up serialization machinery
  constexpr int32_t kInitialSerializationBufferSize = 1024;
  std::vector<uint8_t> serialization_buffer(kInitialSerializationBufferSize);
  xcdr_v2_stream cdr_stream(endianness::little_endian);

  // open output file  
  std::ofstream file_stream;
  file_stream.exceptions(std::ios::failbit | std::ios::badbit);
  file_stream.open(filename, std::ios::out | std::ios::binary);

  // write messages to file  
  // Serialize next message to bytes

  //determine the amount of buffer necessary
  move(cdr_stream, sample, false);

  //resize the buffer to have the required capacity
  serialization_buffer.resize(cdr_stream.position());

  //write to the buffer
  cdr_stream.set_buffer(serialization_buffer.data(), serialization_buffer.size());
  write(cdr_stream, sample, false);

  // write message size to log as int64_t
  const auto msg_size = static_cast<std::streamsize>(cdr_stream.position());
  file_stream.write(reinterpret_cast<const char*>(&msg_size), sizeof(msg_size));

  // write serialized message to log
  file_stream.write(reinterpret_cast<char*>(serialization_buffer.data()), msg_size);
}
