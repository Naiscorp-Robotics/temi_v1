# Tệp cấu hình CMake cho dự án robot TEMI-V1

# Đặt phiên bản tối thiểu của CMake là 3.5
cmake_minimum_required(VERSION 3.5)
# Khai báo tên dự án
project(temi_v1)

# Cài đặt các cảnh báo biên dịch nếu sử dụng trình biên dịch GCC hoặc Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# Thêm các tùy chọn biên dịch để hiển thị cảnh báo chi tiết
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm và yêu cầu các gói phụ thuộc cần thiết cho dự án ROS 2
find_package(ament_cmake REQUIRED)       # Hỗ trợ xây dựng gói CMake của ROS 2
find_package(xacro REQUIRED)             # Hỗ trợ xử lý tệp URDF/xacro
find_package(ament_cmake_python REQUIRED)# Hỗ trợ các gói Python trong ROS 2
find_package(rclpy REQUIRED)             # Thư viện Python của ROS 2

# Cài đặt các thư mục dữ liệu của dự án 
install(
  DIRECTORY launch urdf meshes world models config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Cấu hình kiểm tra chất lượng mã nguồn nếu chế độ kiểm tra được bật
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Các dòng bị ghi chú dưới đây cho phép bỏ qua một số kiểm tra lint
  # Có thể bỏ ghi chú nếu cần thiết
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  
  # Tự động tìm và chạy các bài kiểm tra phụ thuộc
  ament_lint_auto_find_test_dependencies()
endif()

# Hoàn tất cấu hình gói ROS 2
ament_package()