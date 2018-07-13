#include "stream_pb.hpp"

using namespace google::protobuf::io;

ProtobufWriter::ProtobufWriter(std::string const &filename)
    : file(filename, std::ios::out | std::ios::binary) {
  raw_output = new OstreamOutputStream(&file);
  coded_output = new CodedOutputStream(raw_output);
}

ProtobufWriter::~ProtobufWriter() { this->close(); }

void ProtobufWriter::close() {
  if (file.is_open()) {
    delete coded_output;
    delete raw_output;
    file.close();
  }
}

ProtobufReader::ProtobufReader(std::string const &filename)
    : file(filename, std::ios::in | std::ios::binary) {
  raw_input = new IstreamInputStream(&file);
  coded_input = new CodedInputStream(raw_input);
}

ProtobufReader::~ProtobufReader() { this->close(); }

void ProtobufReader::close() {
  if (file.is_open()) {
    delete coded_input;
    delete raw_input;
    file.close();
  }
}