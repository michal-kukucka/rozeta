#include "internal/serial_port.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <string>
#include <utility>

namespace rozeta::internal {
namespace {

Status makeError(ErrorCode code, const std::string& operation, const std::string& detail) {
    return Status::error(code, operation + ": " + detail);
}

std::string windowsErrorMessage(DWORD error) {
    if (error == 0) {
        return "unknown Windows error";
    }

    LPSTR buffer = nullptr;
    DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS;
    DWORD length = ::FormatMessageA(
        flags,
        nullptr,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPSTR>(&buffer),
        0,
        nullptr);
    std::string message = length > 0 && buffer ? std::string(buffer, length) : "Windows error";
    if (buffer) {
        ::LocalFree(buffer);
    }
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    return message;
}

Status lastError(ErrorCode code, const std::string& operation, const std::string& device) {
    std::string message = device.empty() ? operation : operation + "(" + device + ")";
    message += " failed: ";
    message += windowsErrorMessage(::GetLastError());
    return Status::error(code, message);
}

DWORD baudToWin32(int baud) {
    switch (baud) {
        case 4800: return CBR_4800;
        case 9600: return CBR_9600;
        case 19200: return CBR_19200;
        case 38400: return CBR_38400;
        case 57600: return CBR_57600;
        case 115200: return CBR_115200;
        case 128000: return CBR_128000;
        case 230400: return 230400;
        case 460800: return 460800;
        case 921600: return 921600;
        default: return 0;
    }
}

DWORD timeoutToDword(std::chrono::milliseconds timeout) {
    return static_cast<DWORD>(std::min<long long>(timeout.count(), MAXDWORD));
}

ErrorCode openErrorCode(DWORD error) {
    switch (error) {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
        case ERROR_ACCESS_DENIED:
        case ERROR_SHARING_VIOLATION:
        case ERROR_DEVICE_NOT_CONNECTED:
            return ErrorCode::HardwareUnavailable;
        default:
            return ErrorCode::IoError;
    }
}

} // namespace

struct SerialPort::Impl {
    HANDLE handle{INVALID_HANDLE_VALUE};
    SerialPortConfig config{};
};

SerialPort::SerialPort() : impl_(std::make_unique<Impl>()) {}

SerialPort::~SerialPort() {
    close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept = default;

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        close();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Status SerialPort::open(const SerialPortConfig& config) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (config.device.empty()) {
        return makeError(ErrorCode::InvalidArgument, "serial open", "device path is empty");
    }
    if (config.read_timeout.count() < 0 || config.write_timeout.count() < 0) {
        return makeError(ErrorCode::InvalidArgument, "serial open", "timeouts must be non-negative");
    }

    DWORD baud = baudToWin32(config.baud_rate);
    if (baud == 0) {
        return makeError(
            ErrorCode::InvalidArgument,
            "serial open",
            "unsupported baud rate " + std::to_string(config.baud_rate));
    }

    close();

    HANDLE opened = ::CreateFileA(
        config.device.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (opened == INVALID_HANDLE_VALUE) {
        return lastError(openErrorCode(::GetLastError()), "CreateFileA", config.device);
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(opened, &dcb)) {
        Status err = lastError(ErrorCode::IoError, "GetCommState", config.device);
        ::CloseHandle(opened);
        return err;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fAbortOnError = FALSE;

    if (!::SetCommState(opened, &dcb)) {
        Status err = lastError(ErrorCode::IoError, "SetCommState", config.device);
        ::CloseHandle(opened);
        return err;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = timeoutToDword(config.read_timeout);
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = timeoutToDword(config.write_timeout);
    timeouts.WriteTotalTimeoutMultiplier = 0;
    if (!::SetCommTimeouts(opened, &timeouts)) {
        Status err = lastError(ErrorCode::IoError, "SetCommTimeouts", config.device);
        ::CloseHandle(opened);
        return err;
    }

    if (!::PurgeComm(opened, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        Status err = lastError(ErrorCode::IoError, "PurgeComm", config.device);
        ::CloseHandle(opened);
        return err;
    }

    impl_->handle = opened;
    impl_->config = config;
    return Status::okStatus();
}

void SerialPort::close() noexcept {
    if (impl_ && impl_->handle != INVALID_HANDLE_VALUE) {
        ::CloseHandle(impl_->handle);
        impl_->handle = INVALID_HANDLE_VALUE;
    }
}

bool SerialPort::isOpen() const noexcept {
    return impl_ && impl_->handle != INVALID_HANDLE_VALUE;
}

int SerialPort::nativeFd() const noexcept {
    return -1;
}

Status SerialPort::readSome(std::uint8_t* buffer, std::size_t capacity, std::size_t& bytes_read) {
    bytes_read = 0;
    if (!buffer || capacity == 0) {
        return makeError(
            ErrorCode::InvalidArgument,
            "serial read",
            "buffer must be non-null and non-empty");
    }
    if (!isOpen()) {
        return makeError(ErrorCode::HardwareUnavailable, "serial read", "serial port is not open");
    }
    if (capacity > MAXDWORD) {
        return makeError(ErrorCode::InvalidArgument, "serial read", "buffer is too large");
    }

    DWORD count = 0;
    if (!::ReadFile(impl_->handle, buffer, static_cast<DWORD>(capacity), &count, nullptr)) {
        return lastError(ErrorCode::IoError, "ReadFile", impl_->config.device);
    }
    if (count == 0) {
        return makeError(ErrorCode::Timeout, "serial read", "timeout");
    }
    bytes_read = static_cast<std::size_t>(count);
    return Status::okStatus();
}

Status SerialPort::writeAll(const std::uint8_t* data, std::size_t size) {
    if (size == 0) {
        return Status::okStatus();
    }
    if (!data) {
        return makeError(
            ErrorCode::InvalidArgument,
            "serial write",
            "data must be non-null when size is non-zero");
    }
    if (!isOpen()) {
        return makeError(ErrorCode::HardwareUnavailable, "serial write", "serial port is not open");
    }

    std::size_t written = 0;
    while (written < size) {
        std::size_t chunk = std::min<std::size_t>(size - written, MAXDWORD);
        DWORD count = 0;
        if (!::WriteFile(
                impl_->handle,
                data + written,
                static_cast<DWORD>(chunk),
                &count,
                nullptr)) {
            return lastError(ErrorCode::IoError, "WriteFile", impl_->config.device);
        }
        if (count == 0) {
            return makeError(ErrorCode::Timeout, "serial write", "no progress before timeout");
        }
        written += static_cast<std::size_t>(count);
    }
    return Status::okStatus();
}

} // namespace rozeta::internal
