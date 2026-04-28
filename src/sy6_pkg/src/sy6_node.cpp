#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "std_msgs/msg/float64_multi_array.hpp"

const double PI = std::acos(-1.0);
struct AxleState
{
  std::vector<std::vector<double>> coordinate_system_change;
  std::vector<std::vector<double>> coordinate_follow_change;
  std::vector<std::vector<double>> location_real;
  std::vector<std::vector<double>> location_axle;
  double axle_long = 0.0;
  double location_deviation_z = 0.0;
  double location_deviation_y = 0.0;
  double location_deviation_x = 0.0;
  double angle_a = 0.0;
  double angle_c = 0.0;
  double angle_init = 0.0;
  double spin_direction = 1.0;
};

class MatrixNode : public rclcpp::Node
{
public:
  MatrixNode() : Node("sy6_node")
  {
    this->declare_parameter<std::vector<double>>("all_moto_angle_now", std::vector<double>(6, 0.0));
    moto_angle_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("all_moto_angle_now", 10, std::bind(&MatrixNode::moto_angle_callback, this, std::placeholders::_1));
    location_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("location_now", 10);
    RCLCPP_INFO(this->get_logger(), "矩阵节点已启动");
    // motor_angle_now_ ={180.0,125.24,60.24,0.0,115.01};
    axle_init();
    // test_matrix_multiply();
    // test_matrix_inverse();
    // test_transform_matrix_solving(); 
  }

/**
     * 测试变换矩阵求解功能
     */
    void test_transform_matrix_solving() {
        RCLCPP_INFO(this->get_logger(), "=== 变换矩阵求解测试 ===");
        
        // 测试案例1：简单的缩放变换
        std::vector<std::vector<double>> A1 = {
            {0, 1, 0},  // 3个点的x坐标
            {-1, 0, 0},  // 3个点的y坐标  
            {0, 0, 1}   // 3个点的z坐标
        };
        
        // B1 = 2 × A1（缩放变换）
        std::vector<std::vector<double>> B1 = {
            {1, 0, 0},
            {0, 0, 1}, 
            {0, -1, 0}
        };
        
        auto T1 = solve_transform_matrix(A1, B1);
        T1 = matrix_transpose(T1); // 由于求解的是A到B的变换，这里取逆得到B到A的变换
        if (!T1.empty()) {
            RCLCPP_INFO(this->get_logger(), "测试1: 缩放变换矩阵");
            printMatrix(T1, "变换矩阵T");
            
            // double error1 = verify_transform_accuracy(T1, A1, B1);
            // RCLCPP_INFO(this->get_logger(), "均方根误差: %.6f", error1);
        }
        
        // 测试案例2：实际机器人坐标变换
        std::vector<std::vector<double>> source_points = {
            {0, 0, -1},  // x坐标：各轴末端x位置
            {0, 1, 0},                   // y坐标
            {1, 0, 0}  // z坐标
        };
        
        // 假设目标点集（经过某种变换）
        std::vector<std::vector<double>> target_points = {
            {0, -1, 0},
            {0, 0, -1},
            {1, 0, 0}
        };
        
        auto T2 = solve_transform_matrix(source_points, target_points);
        T2 = matrix_transpose(T2);
        if (!T2.empty()) {
            RCLCPP_INFO(this->get_logger(), "测试2: 机器人坐标变换矩阵");
            printMatrix(T2, "变换矩阵T");
            
            // double error2 = verify_transform_accuracy(T2, source_points, target_points);
            // RCLCPP_INFO(this->get_logger(), "均方根误差: %.6f", error2);
            
            // // 验证变换结果
            // auto transformed = Matrix_multiply(T2, source_points);
            // RCLCPP_INFO(this->get_logger(), "变换后的点集:");
            // printMatrix(transformed, "预测值");
            // printMatrix(target_points, "实际值");
        }
    }
  void test_matrix_multiply() {
    // 测试1：单位矩阵乘法
    std::vector<std::vector<double>> identity = {{1,0,0},{0,1,0},{0,0,1}};
    std::vector<std::vector<double>> vector = {{2},{3},{4}};
    auto result = Matrix_multiply(identity, vector);
    RCLCPP_INFO(this->get_logger(), "Identity test: [%.1f, %.1f, %.1f]", 
                result[0][0], result[1][0], result[2][0]);
    
    // 测试2：简单变换
    std::vector<std::vector<double>> transform = {{2,0,0},{0,2,0},{0,0,2}};
    result = Matrix_multiply(transform, vector);
    RCLCPP_INFO(this->get_logger(), "Scale test: [%.1f, %.1f, %.1f]", 
                result[0][0], result[1][0], result[2][0]);
    
    // 测试3：绕Z轴旋转90度
    std::vector<std::vector<double>> rotate90 = {{0,-1,0},{1,0,0},{0,0,1}};
    std::vector<std::vector<double>> point = {{1},{0},{0}};
    result = Matrix_multiply(rotate90, point);
    RCLCPP_INFO(this->get_logger(), "Rotation test: [%.1f, %.1f, %.1f]", 
                result[0][0], result[1][0], result[2][0]);
  }
  // 新增：矩阵求逆测试函数
    void test_matrix_inverse() {
        RCLCPP_INFO(this->get_logger(), "=== 矩阵求逆测试 ===");
        
        // 测试2x2矩阵
        std::vector<std::vector<double>> A2 = {
            {4, 7},
            {2, 6}
        };
        
        try {
            auto invA2 = Matrix_inverse(A2);
            RCLCPP_INFO(this->get_logger(), "2x2矩阵求逆成功");
            printMatrix(invA2, "逆矩阵 A⁻¹");
            
            // 验证 A × A⁻¹ ≈ I
            auto identity_check = Matrix_multiply(A2, invA2);
            RCLCPP_INFO(this->get_logger(), "验证 A × A⁻¹ (应接近单位矩阵):");
            printMatrix(identity_check);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "2x2矩阵求逆失败: %s", e.what());
        }
        
        // 测试3x3矩阵
        std::vector<std::vector<double>> A3 = {
            {0, -1, 0},
            {0, 0, 1},
            {-1, 0, 0}
        };
        
        try {
            auto invA3 = Matrix_inverse(A3);
            RCLCPP_INFO(this->get_logger(), "3x3矩阵求逆成功");
            printMatrix(invA3, "逆矩阵 A⁻¹");
            
            auto identity_check3 = Matrix_multiply(A3, invA3);
            RCLCPP_INFO(this->get_logger(), "验证 A × A⁻¹ (应接近单位矩阵):");
            printMatrix(identity_check3);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "3x3矩阵求逆失败: %s", e.what());
        }
    }
  void axle_init()
  {
    // 初始化每个轴的向量
    for (int i = 0; i < 6; i++) {
      axles_[i].location_axle = std::vector<std::vector<double>>(3, std::vector<double>(1, 0.0));
      axles_[i].location_real = std::vector<std::vector<double>>(3, std::vector<double>(1, 0.0));
    }

    // 轴1初始化
    axles_[0].coordinate_system_change = {{1.0,0.0,0.0},
                                        {0.0,1.0,0.0},
                                        {0.0,0.0,1.0}};
    axles_[0].axle_long = 0.0;
    axles_[0].location_deviation_z = 130.5;
    axles_[0].location_deviation_y = 0.0;
    axles_[0].location_deviation_x = 0.0;
    axles_[0].angle_init = 0.0;
    axles_[0].spin_direction = 1.0;

    // 轴2初始化
    axles_[1].coordinate_system_change = {{0.0,1.0,0.0},
                                        {-1.0,0.0,0.0},
                                        {0.0,0.0,1.0}};
    axles_[1].axle_long = 46.84;
    axles_[1].location_deviation_z = 40;
    axles_[1].location_deviation_y = 0.0;
    axles_[1].location_deviation_x = 46.84;
    axles_[1].angle_init = 270.0;
    axles_[1].spin_direction = 1.0;

    // 轴3初始化
    // axles_[2].coordinate_system_change = {{1.0,0.0,0.0},
    //                                     {0.0,0.0,1.0},
    //                                     {0.0,-1.0,0.0}};
    // axles_[2].coordinate_system_change = {{0.0,-1.0,0.0},
    //                                     {0.0,0.0,1.0},
    //                                     {-1.0,0.0,0.0}};
    axles_[2].coordinate_system_change = {{0.0,0.0,-1.0},
                                        {1.0,0.0,0.0},
                                        {0.0,-1.0,0.0}}; 
    axles_[2].axle_long = 180.0;
    axles_[2].location_deviation_z = 4.0;
    axles_[2].location_deviation_y = 0.0;
    axles_[2].location_deviation_x = 180.0;
    axles_[2].angle_init = 0.0;
    axles_[2].spin_direction = 1.0;
    
    // 轴4初始化
    // axles_[3].coordinate_system_change = {{0.0,1.0,0.0},
    //                                     {0.0,0.0,1.0},
    //                                     {1.0,0.0,0.0}};
    axles_[3].coordinate_system_change = {{0.0,1.0,0.0},
                                        {-1.0,0.0,0.0},
                                        {0.0,0.0,1.0}};

    axles_[3].axle_long = 168.0;
    axles_[3].location_deviation_z = 35.5;
    axles_[3].location_deviation_y = -29.0;
    axles_[3].location_deviation_x = 168.0;
    axles_[3].angle_init = 0.0;
    axles_[3].spin_direction = -1.0;
          
    // 轴5初始化
    // axles_[4].coordinate_system_change = {{0.0,0.0,-1.0},
    //                                     {0.0,1.0,0.0},
    //                                     {1.0,0.0,0.0}};
    axles_[4].coordinate_system_change = {{1.0,0.0,0.0},
                                        {0.0,0.0,-1.0},
                                        {0.0,1.0,0.0}};

    axles_[4].axle_long = 50.5;
    axles_[4].location_deviation_z = 98.5;
    axles_[4].location_deviation_y = 50.5;
    axles_[4].location_deviation_x = 0.0;
    axles_[4].angle_init = 0.0; 
    axles_[4].spin_direction = 1.0;
          
    // 轴6初始化
    // axles_[5].coordinate_system_change = {{0.0,-1.0,0.0},
    //                                     {0.0,0.0,-1.0},
    //                                     {1.0,0.0,0.0}};
    axles_[5].coordinate_system_change = {{1.0,0.0,0.0},
                                        {0.0,0.0,-1.0},
                                        {0.0,1.0,0.0}};

    axles_[5].axle_long = 143.0;
    axles_[5].location_deviation_z = 43.16;
    axles_[5].location_deviation_y = 0.0;
    axles_[5].location_deviation_x = 143.0;
    axles_[5].angle_init = 0.0; 
    axles_[5].spin_direction = -1.0;           

    // motor_angle_now_ ={135.0,0.0,0.0,0.0,0.0,0.0};
    // 检查参数大小
    if (motor_angle_now_.size() < 5) {
      RCLCPP_ERROR(this->get_logger(), "参数'all_moto_angle_now'大小不足，需要至少5个值");
      return;
    }

    // 初始化轴0
    axles_[0].angle_c = 0.0;
    axles_[0].angle_a = (axles_[0].angle_init + axles_[0].angle_c * axles_[0].spin_direction) / 180 * PI;
    
    // 设置轴0的位置向量
    axles_[0].location_axle[0][0] = axles_[0].location_deviation_x;
    axles_[0].location_axle[1][0] = axles_[0].location_deviation_y;
    axles_[0].location_axle[2][0] = axles_[0].location_deviation_z;
    
    // 应用坐标变换
    axles_[0].location_axle = Matrix_multiply(axles_[0].coordinate_system_change, axles_[0].location_axle);
    axles_[0].location_real = axles_[0].location_axle;

    // 打印轴0的坐标
    RCLCPP_DEBUG(this->get_logger(), "axles_[0] location_real: x=%.3f, y=%.3f, z=%.3f",
                 axles_[0].location_real[0][0],
                 axles_[0].location_real[1][0],
                 axles_[0].location_real[2][0]);

    // 初始化其他轴
    
    for(int i = 1; i < 6; i++) {
    axles_[i].angle_c = motor_angle_now_[i-1];
    axles_[i].angle_a = (axles_[i].angle_init + axles_[i].angle_c * axles_[i].spin_direction) / 180 * PI;

    // 设置轴的位置向量
    axles_[i].location_axle[0][0] = axles_[i].location_deviation_x;
    axles_[i].location_axle[1][0] = axles_[i].location_deviation_y;
    axles_[i].location_axle[2][0] = axles_[i].location_deviation_z;

    
    axles_[i].coordinate_system_change = Matrix_multiply(axles_[i-1].coordinate_system_change, axles_[i].coordinate_system_change);

    axles_[i].coordinate_system_change = coordinate_transform(axles_[i].coordinate_system_change,0,0,axles_[i].angle_c,1,1,axles_[i].spin_direction);
    
    // 应用坐标变换
    axles_[i].location_axle = Matrix_multiply(axles_[i].coordinate_system_change, axles_[i].location_axle);
    
    // 计算实际位置
    axles_[i].location_real = Matrix_add(axles_[i-1].location_real, axles_[i].location_axle);
    
    // 打印轴的坐标
    RCLCPP_DEBUG(this->get_logger(), "axles_[%d] location_real: x=%.3f, y=%.3f, z=%.3f",
                 i,
                 axles_[i].location_real[0][0],
                 axles_[i].location_real[1][0],
                 axles_[i].location_real[2][0]);
    }
    auto location_message = std_msgs::msg::Float64MultiArray();
    location_message.data.resize(3);
    for(int j=0 ;j<3 ;j++){
        location_message.data[j] = axles_[5].location_real[j][0];
    }
    location_publisher_->publish(location_message);
  }
    

private:
  void moto_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 5) {
      RCLCPP_ERROR(this->get_logger(), "接收到的'all_moto_angle_now'数据大小不足，需要至少5个值");
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    motor_angle_now_ = msg->data;
    axle_init();
  }
 /**
     * 求解从矩阵A到矩阵B的变换矩阵T（最小二乘法）
     * 方程：T × A = B，求解 T = B × Aᵀ × (A × Aᵀ)⁻¹
     * @param A 源矩阵（3×N，每列是一个点）
     * @param B 目标矩阵（3×N，每列是一个点）
     * @return 变换矩阵T（3×3）
     */
    std::vector<std::vector<double>> solve_transform_matrix(
        const std::vector<std::vector<double>>& A,
        const std::vector<std::vector<double>>& B) {
        
        // 检查输入有效性
        if (!isMatrix(A) || !isMatrix(B)) {
            RCLCPP_ERROR(this->get_logger(), "输入数据不是矩阵");
            return {};
        }
        
        if (A.size() != 3 || B.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "点集必须是3维（3行）");
            return {};
        }
        
        if (A[0].size() != B[0].size()) {
            RCLCPP_ERROR(this->get_logger(), "点数量不匹配: A有%zu个点, B有%zu个点", 
                        A[0].size(), B[0].size());
            return {};
        }
        
        size_t num_points = A[0].size();
        if (num_points < 3) {
            RCLCPP_WARN(this->get_logger(), "点数量较少(%zu)，结果可能不准确", num_points);
        }
        
        try {
            // 1. 计算 A × Aᵀ（3×3矩阵）
            auto A_transpose = matrix_transpose(A);
            auto A_Atrans = Matrix_multiply(A, A_transpose);
            
            // 2. 计算逆矩阵 (A × Aᵀ)⁻¹
            auto inv_A_Atrans = Matrix_inverse(A_Atrans);
            
            // 3. 计算 B × Aᵀ（3×3矩阵）
            auto B_Atrans = Matrix_multiply(B, A_transpose);
            
            // 4. 计算 T = (B × Aᵀ) × (A × Aᵀ)⁻¹
            auto T = Matrix_multiply(B_Atrans, inv_A_Atrans);
            
            return T;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "求解变换矩阵失败: %s", e.what());
            return {};
        }
    }
    
    /**
     * 验证变换矩阵的准确性
     * @return 均方根误差
     */
    double verify_transform_accuracy(const std::vector<std::vector<double>>& T,
                                   const std::vector<std::vector<double>>& A,
                                   const std::vector<std::vector<double>>& B) {
        if (T.empty() || A.empty() || B.empty()) return -1.0;
        
        auto B_predicted = Matrix_multiply(T, A);
        
        double total_error = 0.0;
        int point_count = static_cast<int>(A[0].size());  // 显式转换
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < point_count; j++) {  // 现在类型一致
                double error = B[i][j] - B_predicted[i][j];
                total_error += error * error;
            }
        }
        
        double rmse = std::sqrt(total_error / (3.0 * point_count));
        return rmse;
    }
    // 矩阵转置
    std::vector<std::vector<double>> matrix_transpose(const std::vector<std::vector<double>>& matrix) {
        if (!isMatrix(matrix)) {
            RCLCPP_ERROR(this->get_logger(), "输入数据不是矩阵");
            return {};
        }
        int rows = matrix.size();
        int cols = matrix[0].size();
        std::vector<std::vector<double>> result(cols, std::vector<double>(rows, 0.0));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j][i] = matrix[i][j];
            }
        }
        return result;
    }
    // 打印矩阵的辅助函数
    void printMatrix(const std::vector<std::vector<double>>& matrix, const std::string& name = "") {
        if (!name.empty()) {
            RCLCPP_INFO(this->get_logger(), "%s:", name.c_str());
        }
        
        for (const auto& row : matrix) {
            std::string row_str;
            for (size_t j = 0; j < row.size(); j++) {
                row_str += std::to_string(row[j]);
                if (j < row.size() - 1) row_str += ", ";
            }
            RCLCPP_INFO(this->get_logger(), "[%s]", row_str.c_str());
        }
    }
    // 检查矩阵是否为方阵
    bool isSquareMatrix(const std::vector<std::vector<double>>& matrix) {
        if (matrix.empty()) return false;
        size_t n = matrix.size();
        for (const auto& row : matrix) {
            if (row.size() != n) return false;
        }
        return true;
    }

    // 计算矩阵行列式（递归实现）
    double calculateDeterminant(const std::vector<std::vector<double>>& matrix) {
        if (!isSquareMatrix(matrix)) {
            throw std::invalid_argument("矩阵必须是方阵才能计算行列式");
        }
        
        int n = matrix.size();
        
        // 基本情况
        if (n == 1) return matrix[0][0];
        if (n == 2) {
            return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        }
        
        double det = 0.0;
        // 按第一行展开计算行列式
        for (int col = 0; col < n; col++) {
            // 创建余子式矩阵（去掉第0行第col列）
            std::vector<std::vector<double>> minor = createMinor(matrix, 0, col);
            
            // 计算代数余子式：(-1)^(行+列) * 余子式行列式
            double sign = (col % 2 == 0) ? 1.0 : -1.0;
            double cofactor = sign * calculateDeterminant(minor);
            
            det += matrix[0][col] * cofactor;
        }
        
        return det;
    }
    
    // 创建余子式矩阵（去掉指定行和列）
    std::vector<std::vector<double>> createMinor(const std::vector<std::vector<double>>& matrix, 
                                               int rowToRemove, int colToRemove) {
        int n = matrix.size();
        std::vector<std::vector<double>> minor(n - 1, std::vector<double>(n - 1));
        
        int minorRow = 0;
        for (int i = 0; i < n; i++) {
            if (i == rowToRemove) continue;
            int minorCol = 0;
            for (int j = 0; j < n; j++) {
                if (j == colToRemove) continue;
                minor[minorRow][minorCol++] = matrix[i][j];
            }
            minorRow++;
        }
        return minor;
    }
    
    // 计算伴随矩阵
    std::vector<std::vector<double>> calculateAdjoint(const std::vector<std::vector<double>>& matrix) {
        if (!isSquareMatrix(matrix)) {
            throw std::invalid_argument("矩阵必须是方阵才能计算伴随矩阵");
        }
        
        int n = matrix.size();
        std::vector<std::vector<double>> adjoint(n, std::vector<double>(n));
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                // 创建余子式矩阵（去掉第i行第j列）
                std::vector<std::vector<double>> minor = createMinor(matrix, i, j);
                
                // 计算代数余子式（注意转置：伴随矩阵是代数余子式矩阵的转置）
                double sign = ((i + j) % 2 == 0) ? 1.0 : -1.0;
                double minorDet = calculateDeterminant(minor);
                adjoint[j][i] = sign * minorDet; // 转置：adjoint[j][i] = C_ij
            }
        }
        
        return adjoint;
    }

    // 使用伴随矩阵法求逆
    std::vector<std::vector<double>> Matrix_inverse(const std::vector<std::vector<double>>& matrix) {
        if (!isSquareMatrix(matrix)) {
            throw std::invalid_argument("矩阵必须是方阵才能求逆");
        }
        
        int n = matrix.size();
        double det = calculateDeterminant(matrix);
        
        if (std::fabs(det) < 1e-12) {
            throw std::runtime_error("矩阵不可逆，行列式为零");
        }
        
        std::vector<std::vector<double>> adjoint = calculateAdjoint(matrix);
        
        // 计算逆矩阵：A⁻¹ = adj(A) / det(A)
        std::vector<std::vector<double>> inverse(n, std::vector<double>(n));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                inverse[i][j] = adjoint[i][j] / det;
            }
        }
        
        return inverse;
    }
  bool isMatrix(const std::vector<std::vector<double>>& data) {
    if (data.empty()) {
        return false;
    }
    size_t numCols = data[0].size();
    for (size_t i = 1; i < data.size(); ++i) {
        if (data[i].size() != numCols) {
            return false;
        }
    }
    return true;
  }

  std::vector<std::vector<double>> Matrix_add(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    if(!isMatrix(A) || !isMatrix(B)) {
      RCLCPP_ERROR(this->get_logger(), "输入数据不是矩阵");
      return {};
    }
    int rows_a = A.size();
    int cols_a = A[0].size();
    int rows_b = B.size();
    int cols_b = B[0].size();
    
    if(rows_a != rows_b || cols_a != cols_b) {
      RCLCPP_ERROR(this->get_logger(), "矩阵维度不匹配，无法相加");
      return {};
    }
    
    std::vector<std::vector<double>> C(rows_a, std::vector<double>(cols_a, 0.0));
    for(int i = 0; i < rows_a; i++) {
      for(int j = 0; j < cols_a; j++) {
        C[i][j] = A[i][j] + B[i][j];
      }
    }
    return C;
  }

std::vector<std::vector<double>> coordinate_transform(
    const std::vector<std::vector<double>>& points_3d,
    double angle_x, double angle_y, double angle_z,
    int direction_x, int direction_y, int direction_z)
{
    if(points_3d.empty() || points_3d[0].size() != 3) {
        // RCLCPP_ERROR(this->get_logger(), "输入数据必须是Nx3矩阵");
        std::cerr << "输入数据必须是Nx3矩阵" << std::endl;
        return {};
    }

    // 构造基本旋转矩阵
    double cosX = std::cos(direction_x * angle_x / 180.0 * PI);
    double sinX = std::sin(direction_x * angle_x / 180.0 * PI);
    
    double cosY = std::cos(direction_y * angle_y / 180.0 * PI);
    double sinY = std::sin(direction_y * angle_y / 180.0 * PI);

    double cosZ = std::cos(direction_z * angle_z / 180.0 * PI);
    double sinZ = std::sin(direction_z * angle_z / 180.0 * PI);
    std::vector<std::vector<double>> rot_x = {
        {1.0, 0.0, 0.0},
        {0.0, cosX, sinX},
        {0.0, -sinX, cosX}
    };


    std::vector<std::vector<double>> rot_y = {
        {cosY, 0.0, sinY},
        {0.0, 1.0, 0.0},
        {-sinY, 0.0, cosY}
    };

    std::vector<std::vector<double>> rot_z = {
        {cosZ, sinZ, 0.0},
        {-sinZ, cosZ, 0.0},
        {0.0, 0.0, 1.0}
    };

    // 自身坐标系旋转：X * Y * Z
    std::vector<std::vector<double>> temp = Matrix_multiply(rot_x, rot_y);
    std::vector<std::vector<double>> combined_rot = Matrix_multiply(temp, rot_z);
    // 将组合变换矩阵应用于所有点（之前误用 rot_z，丢失了 X/Y 轴旋转）
    return Matrix_multiply(points_3d, combined_rot);
}


  std::vector<std::vector<double>> Matrix_multiply(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    if(!isMatrix(A) || !isMatrix(B)) {
      RCLCPP_ERROR(this->get_logger(), "输入数据不是矩阵");
      return {};
    }
    int rows_a = A.size();
    int cols_a = A[0].size();
    int rows_b = B.size();
    int cols_b = B[0].size();
    
    if(cols_a != rows_b) {
      RCLCPP_ERROR(this->get_logger(), "矩阵维度不匹配，无法相乘");
      return {};
    }
    
    std::vector<std::vector<double>> C(rows_a, std::vector<double>(cols_b, 0.0));
    for(int i = 0; i < rows_a; i++) {
      for(int j = 0; j < cols_b; j++) {
        for(int k = 0; k < cols_a; k++) {
          C[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return C;
  }

  std::vector<std::vector<double>> Matrix_multiply_constant(double constant_number, const std::vector<std::vector<double>>& A) {
    if(!isMatrix(A)) {
      RCLCPP_ERROR(this->get_logger(), "输入数据不是矩阵");
      return {};
    }
    int rows_a = A.size();
    int cols_a = A[0].size();
    
    std::vector<std::vector<double>> C(rows_a, std::vector<double>(cols_a, 0.0));
    for(int i = 0; i < rows_a; i++) {
        for(int j = 0; j < cols_a; j++) {
          C[i][j] = A[i][j] * constant_number;
        }
    }
    return C;
  }

  std::vector<AxleState> axles_{std::vector<AxleState>(6)};
  std::vector<double> motor_angle_now_;
  std::mutex state_mutex_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr moto_angle_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr location_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MatrixNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}