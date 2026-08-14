#include "PlayerLineSources.h"

#include <cstring>

#include "libs/FirmwareFileSystem.h"

LineReadResult FileLineSource::read(char* buffer, std::size_t size) {
  if (file_ == nullptr || buffer == nullptr || size == 0) return LineReadResult::error;
  if (fwfs::fgets(buffer, size, file_) != nullptr) return LineReadResult::data;
  return fwfs::feof(file_) ? LineReadResult::end : LineReadResult::error;
}

bool FileLineSource::at_end() const { return file_ != nullptr && fwfs::feof(file_); }

unsigned long FileLineSource::position() const {
  if (file_ == nullptr) return 0;
  const long offset = fwfs::ftell(file_);
  return offset > 0 ? static_cast<unsigned long>(offset) : 0;
}

void FileLineSource::close() {
  if (file_ != nullptr) {
    fwfs::fclose(file_);
    file_ = nullptr;
  }
}

StreamedJobBuffer::StreamedJobBuffer(Line* storage, std::size_t capacity) : storage_(storage), capacity_(capacity) {}

void StreamedJobBuffer::begin(uint16_t filename_crc, uint32_t first_line, unsigned long byte_position) {
  close();
  filename_crc_ = filename_crc;
  consumed_bytes_ = byte_position;
  next_expected_line_ = first_line;
  open_ = true;
}

StreamedJobBuffer::AppendResult StreamedJobBuffer::append_lines(uint16_t filename_crc, uint32_t first_line,
                                                                const uint8_t* data, std::size_t size) {
  if (!open_ || eof_) return AppendResult::not_receiving;
  if (data == nullptr || size == 0) return AppendResult::invalid_data;
  if (filename_crc != filename_crc_) return AppendResult::wrong_file;
  if (first_line != next_expected_line_) return AppendResult::unexpected_line;

  std::size_t lines = 0;
  std::size_t begin = 0;
  for (std::size_t i = 0; i <= size; ++i) {
    const bool newline = i < size && data[i] == '\n';
    if (!newline && i != size) continue;
    if (i == size && begin == i) break;

    const std::size_t length = i - begin + (newline ? 1 : 0);
    if (length >= line_capacity) return AppendResult::line_too_long;
    ++lines;
    begin = i + (newline ? 1 : 0);
  }

  if (lines > available()) return AppendResult::queue_full;

  begin = 0;
  for (std::size_t i = 0; i <= size; ++i) {
    const bool newline = i < size && data[i] == '\n';
    if (!newline && i != size) continue;
    if (i == size && begin == i) break;

    const std::size_t length = i - begin + (newline ? 1 : 0);
    Line& slot = storage_[head_];
    std::memcpy(slot.text, data + begin, length);
    slot.text[length] = '\0';
    slot.length = static_cast<uint8_t>(length);
    head_ = (head_ + 1) % capacity_;
    ++count_;
    ++next_expected_line_;
    begin = i + (newline ? 1 : 0);
  }

  return AppendResult::success;
}

bool StreamedJobBuffer::mark_end(uint16_t filename_crc) {
  if (!open_ || filename_crc != filename_crc_) return false;
  eof_ = true;
  return true;
}

LineReadResult StreamedJobBuffer::read(char* buffer, std::size_t size) {
  if (!open_) return LineReadResult::error;
  if (count_ == 0) return eof_ ? LineReadResult::end : LineReadResult::waiting;

  Line& slot = storage_[tail_];
  if (buffer == nullptr || size <= slot.length) return LineReadResult::error;
  std::memcpy(buffer, slot.text, static_cast<std::size_t>(slot.length) + 1);
  consumed_bytes_ += slot.length;
  slot.length = 0;
  slot.text[0] = '\0';
  tail_ = (tail_ + 1) % capacity_;
  --count_;
  return LineReadResult::data;
}

bool StreamedJobBuffer::at_end() const { return open_ && eof_ && count_ == 0; }

void StreamedJobBuffer::close() {
  head_ = 0;
  tail_ = 0;
  count_ = 0;
  filename_crc_ = 0;
  consumed_bytes_ = 0;
  next_expected_line_ = 0;
  open_ = false;
  eof_ = false;
}
