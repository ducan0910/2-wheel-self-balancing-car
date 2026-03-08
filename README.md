# 2 wheel self balancing car

Xây dựng một con xe 2 bánh có thể tự cân bằng trên các địa hình phẳng, có thể tự cân bằng lại dù có tác động không vượt ngưỡng. Bằng cách sử dụng bộ điều khiển PID, từ kết quả đo được của MPU6050 trên xe, MCU sẽ suy tính ra được PWM hợp lý cần xuất cho motor làm sao để xe không bị ngã.  

## Tính năng chính

- Xử lý góc nghiêng bằng Complementary Filter.
- Sử dụng thuật toán PID để tính toán PWM hợp lý.
- Bù ma sát tĩnh (Deadzone) cho động cơ DC motor vàng.

## Phần cứng
STT,Tên linh kiện,Số lượng,Mô tả
1,ESP32 DevKit V1,1,"Vi điều khiển chính, 2 nhân 240MHz."
2,MPU6050,1,Cảm biến gia tốc và con quay hồi chuyển.
3,MX1508,1,Mạch cầu H điều khiển 2 động cơ DC.
4,TT Motor,2,"Động cơ vàng, tỷ lệ 1:48."
5,Pin 18650,2,Pin Li-ion 7.4V cấp nguồn hệ thống.
6,Khung xe,1,Mica hoặc in 3D tầng tầng.
