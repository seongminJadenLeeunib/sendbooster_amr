#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <sys/select.h>

class SerialPort {
public:
    SerialPort(const std::string& portName) : m_portName(portName), m_fd(-1) {}

    bool openPort() {
        // 시리얼 포트 열기
        m_fd = open(m_portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_fd == -1) {
            std::cerr << "Failed to open port " << m_portName << ": " << strerror(errno) << std::endl;
            return false;
        }

        // 포트 설정
        struct termios options;
        tcgetattr(m_fd, &options);
        
        // 시리얼 포트 설정: 115200bps, 8 비트 데이터, 패리티 없음, 1 비트 정지 비트
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        
        options.c_cflag &= ~PARENB;  // 패리티 비트 비활성화
        options.c_cflag &= ~CSTOPB;  // 1 비트 정지 비트
        options.c_cflag &= ~CSIZE;   // 데이터 비트 마스크
        options.c_cflag |= CS8;      // 8 비트 데이터 비트
        
        options.c_cflag &= ~CRTSCTS; // 하드웨어 흐름 제어 비활성화
        options.c_cflag |= CREAD | CLOCAL; // 수신 및 로컬 포트 설정

        // 시리얼 포트의 입력과 출력을 비동기식으로 설정
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
        options.c_oflag &= ~OPOST; // 출력 설정

        tcsetattr(m_fd, TCSANOW, &options); // 설정 적용

        return true;
    }

    void closePort() {
        if (m_fd != -1) {
            close(m_fd);
            m_fd = -1;
        }
    }

    bool writeData(const std::string& data) {
        if (m_fd == -1) {
            std::cerr << "Port not opened!" << std::endl;
            return false;
        }

        // 바이트 문자열로 전송
        ssize_t bytesWritten = write(m_fd, data.c_str(), data.length());
        if (bytesWritten == -1) {
            std::cerr << "Failed to write data!" << std::endl;
            return false;
        }
        return true;
    }

    std::string readData() {
        if (m_fd == -1) {
            std::cerr << "Port not opened!" << std::endl;
            return "";
        }

        char buffer[256];
        memset(buffer, 0, sizeof(buffer));

        // 수신 대기 시간 설정
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(m_fd, &readfds);

        timeout.tv_sec = 1;  // 1초 대기
        timeout.tv_usec = 0;

        // select()를 사용하여 포트에 데이터가 있는지 확인
        int result = select(m_fd + 1, &readfds, NULL, NULL, &timeout);
        if (result == -1) {
            std::cerr << "select() error: " << strerror(errno) << std::endl;
            return "";
        } else if (result == 0) {
            std::cerr << "No data available!" << std::endl;
            return "";
        }

        ssize_t bytesRead = read(m_fd, buffer, sizeof(buffer) - 1);
        if (bytesRead == -1) {
            std::cerr << "Failed to read data!" << std::endl;
            return "";
        }

        return std::string(buffer);
    }

private:
    std::string m_portName;
    int m_fd;
};

int main() {
    // 시리얼 포트 이름 (예: /dev/ttyUSB0)
    SerialPort serial("/dev/ttyUSB0");

    // 포트 열기
    if (!serial.openPort()) {
        return -1;
    }

    // 데이터 쓰기
    std::string dataToSend = "ss=4\n";  // 데이터 전송
    if (serial.writeData(dataToSend)) {
        std::cout << "Data sent: " << dataToSend << std::endl;
    }

    // 데이터 읽기 (while 루프 추가)
    std::string receivedData;
    while (true) {
        receivedData = serial.readData();
        
        // 받은 데이터가 비어 있지 않으면 출력하고, 그 후에 계속 데이터를 읽기
        if (!receivedData.empty()) {
            std::cout << "Data received: " << receivedData << std::endl;
        }
        // 데이터를 한 번 읽은 후 계속해서 기다려서 읽을 수 있게 함
        // 예를 들어, 1초마다 계속 읽도록 설정 (혹은 다른 조건을 설정할 수 있음)
        sleep(0.1);  // 1초 대기
    }

    // 포트 닫기
    serial.closePort();

    return 0;
}
