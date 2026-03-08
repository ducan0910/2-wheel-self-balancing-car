# 2 wheel self balancing car

Xây dựng một con xe 2 bánh có thể tự cân bằng trên các địa hình phẳng, có thể tự cân bằng lại dù có tác động không vượt ngưỡng. Bằng cách sử dụng bộ điều khiển PID, từ kết quả đo được của MPU6050 trên xe, MCU sẽ suy tính ra được PWM hợp lý cần xuất cho motor làm sao để xe không bị ngã.  

## Tính năng chính

- Xử lý góc nghiêng bằng Complementary Filter.
- Sử dụng thuật toán PID để tính toán PWM hợp lý.
- Bù ma sát tĩnh (Deadzone) cho động cơ DC vàng.

## Phần cứng

- ESP32 DevKit V1.
- MPU6050.
- MX1508 + 2 động cơ DC vàng.
- 2 pin Li-ion 18650.

| MPU6050 Pin | ESP32 Pin | MX1508 Pin | ESP32 Pin |
|-------------|-----------|-----------|-----------|
| VCC | 3.3V | VCC | Vin (7.4V) |
| GND | GND | GND | GND |
| SDA | GPIO21 | IN1 | GPIO32 |
| SCL | GPIO22 | IN2 | GPIO33 |
|  |  | IN3 | GPIO25 |
|  |  | IN4 | GPIO26 |

## Kiến trúc phần mềm
- main/: Chứa app_main.c quản lý Task điều khiển chính.
- components/my_mpu6050/: Driver đọc dữ liệu I2C và tính toán bộ lọc bù.
- components/my_mx1508/: Driver điều khiển PWM và xử lý Deadzone/Slew Rate.
- components/my_pid/: Thư viện PID.

## Công thức sử dụng

### Complementary Filter

$\text{Angle} = \alpha \cdot (\text{Angle} + \text{Gyro} \cdot dt) + (1 - \alpha) \cdot \text{Accel}$

### Bộ điều khiển PID

$$u(t) = MV(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{d}{dt} e(t)$$

<img width="377" height="44" alt="image" src="https://github.com/user-attachments/assets/72ccabbd-12ef-40fd-85e9-e0cf002039e3" />
