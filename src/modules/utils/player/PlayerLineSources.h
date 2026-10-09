#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

enum class LineReadResult { data, waiting, end, error };

class FileLineSource {
 public:
  void attach(FILE* file) { file_ = file; }
  FILE* file() const { return file_; }

  LineReadResult read(char* buffer, std::size_t size);
  bool at_end() const;
  unsigned long position() const;
  void close();
  bool is_open() const { return file_ != nullptr; }

 private:
  FILE* file_ = nullptr;
};

class StreamedJobBuffer {
 public:
  static constexpr std::size_t line_capacity = 130;

  struct Line {
    char text[line_capacity];
    uint8_t length;
  };

  enum class AppendResult {
    success,
    wrong_file,
    unexpected_line,
    line_too_long,
    queue_full,
    not_receiving,
    invalid_data,
  };

  StreamedJobBuffer() = default;
  StreamedJobBuffer(Line* storage, std::size_t capacity);

  void begin(uint16_t filename_crc, uint32_t first_line = 0, unsigned long byte_position = 0);
  AppendResult append_lines(uint16_t filename_crc, uint32_t first_line, const uint8_t* data, std::size_t size);
  bool mark_end(uint16_t filename_crc);

  LineReadResult read(char* buffer, std::size_t size);
  bool at_end() const;
  unsigned long position() const { return consumed_bytes_; }
  void close();

  bool is_open() const { return open_; }
  uint32_t next_expected_line() const { return next_expected_line_; }
  std::size_t queued() const { return count_; }
  std::size_t available() const { return capacity_ - count_; }

 private:
  Line* storage_ = nullptr;
  std::size_t capacity_ = 0;
  std::size_t head_ = 0;
  std::size_t tail_ = 0;
  std::size_t count_ = 0;
  uint16_t filename_crc_ = 0;
  unsigned long consumed_bytes_ = 0;
  uint32_t next_expected_line_ = 0;
  bool open_ = false;
  bool eof_ = false;
};
