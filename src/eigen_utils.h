/**
 * eigen_utils.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 1, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <exception>  // std::invalid_argument
#include <string>     // std::string, std::to_string
#include <sstream>    // std::stringstream

#include <Eigen/Eigen>

namespace EigenUtils {

/**
 * Decode an Eigen matrix from Matlab format:
 *
 * Usage:
 *   Eigen::Vector3d x = DecodeMatlab<Eigen::Vector3d>("1 2 3");
 *   Eigen::MatrixXd A = DecodeMatlab<Eigen::MatrixXd>("1 2 3; 4 5 6");
 */
template<typename Derived>
Derived DecodeMatlab(const std::string& str);

/**
 * Encode an Eigen matrix to Matlab format:
 *
 * Usage:
 *   std::string x = EncodeMatlab(Eigen::Vector3d(1, 2, 3));    // "1 2 3"
 *   std::string A = EncodeMatlab(Eigen::Matrix2d(1, 2, 3, 4)); // "1 2; 3 4"
 */
template<typename Derived>
std::string EncodeMatlab(const Eigen::DenseBase<Derived>& matrix);

/**
 * Decode an Eigen matrix from Json format:
 *
 * Usage:
 *   Eigen::Vector3d x = DecodeJson<Eigen::Vector3d>("[1, 2, 3]");
 *   Eigen::MatrixXd A = DecodeJson<Eigen::MatrixXd>("[[1, 2, 3], [4, 5, 6]]");
 */
template<typename Derived>
Derived DecodeJson(const std::string& str);

/**
 * Encode an Eigen matrix to Json format:
 *
 * Usage:
 *   std::string x = EncodeMatlab(Eigen::Vector3d(1, 2, 3));    // "[1, 2, 3]"
 *   std::string A = EncodeMatlab(Eigen::Matrix2d(1, 2, 3, 4)); // "[[1, 2], [3, 4]]"
 */
template<typename Derived>
std::string EncodeJson(const Eigen::DenseBase<Derived>& matrix);

template<typename Derived>
Derived DecodeMatlab(const std::string& str) {
  std::string str_local = str;

  // Create matrix to return
  Derived matrix;
  size_t num_cols = matrix.cols();
  size_t num_rows = matrix.rows();
  typename Derived::Scalar eof;  // Used to check for eof and print Matrix scalar type

  // Count number of columns
  size_t idx_col_end = 0;
  if (num_cols == 0 || (num_cols == 1 && num_rows == 0)) {
    num_cols = 0;
    size_t idx = 0;
    idx_col_end = str.find_first_of(';');
    while (idx < idx_col_end) {
      // Skip over extra whitespace
      idx = str.find_first_not_of(' ', idx);
      if (idx >= idx_col_end) break;

      // Find next delimiter
      if (str[idx] == ';') break;
      idx = str.find_first_of(' ', idx + 1);
      ++num_cols;
    }
  }

  // Count number of rows
  if (num_rows == 0) {
    size_t idx = idx_col_end;
    if (idx_col_end != 0) {  // First row already traversed
      idx = idx_col_end;
      num_rows = 1;
    }
    while (idx != std::string::npos) {
      // Clear semicolons as we go
      if (idx != 0) str_local[idx] = ' ';

      // Find next delimiter
      idx = str.find_first_of(';', idx + 1);
      ++num_rows;
    }
    // Ignore trailing semicolon
    idx = str.find_last_not_of(' ');
    if (str[idx] == ';') --num_rows;
  } else {
    // Clear remaining semicolons
    for (size_t idx = idx_col_end; idx < str.size(); idx++) {
      if (str[idx] == ';') str_local[idx] = ' ';
    }
  }

  // Check number of rows and columns
  if (num_cols == 0)
    throw std::invalid_argument(
        "DecodeMatlab(): Failed to decode Eigen::MatrixX" +
        std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
        ", " + std::to_string(num_cols) + ") from: (" + str + ").");
  if (num_rows == 1) {
    // Convert to vector
    num_rows = num_cols;
    num_cols = 1;
  }
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    matrix.resize(num_rows, num_cols);
  }

  // Parse matrix
  std::stringstream ss(str_local);
  for (size_t i = 0; i < num_rows; ++i) {
    for (size_t j = 0; j < num_cols; ++j) {
      ss >> matrix(i,j);
      if (ss.fail()) {
        throw std::invalid_argument(
            "DecodeMatlab(): Failed to decode Eigen::MatrixX" +
            std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
            ", " + std::to_string(num_cols) + ") from: (" + str + ").");
      }
    }
  }

  // Make sure there are no numbers left
  ss >> eof;
  if (!ss.fail()) {
    throw std::invalid_argument(
        "DecodeMatlab(): Failed to decode Eigen::MatrixX" +
        std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
        ", " + std::to_string(num_cols) + ") from: (" + str + ").");
  }

  return matrix;
}

template<typename Derived>
std::string EncodeMatlab(const Eigen::DenseBase<Derived>& matrix) {
  std::stringstream ss;
  ss.precision(std::numeric_limits<typename Derived::Scalar>::digits10);
  if (matrix.cols() == 1) { // Column vector
    // [[1],[2],[3],[4]] => "1 2 3 4"
    for (int i = 0; i < matrix.rows(); ++i) {
      if (i > 0) ss << " ";
      ss << matrix(i);
    }
  } else { // matrix
    // [1,2,3,4]     => "1 2 3 4"
    // [[1,2],[3,4]] => "1 2; 3 4"
    for (int i = 0; i < matrix.rows(); ++i) {
      if (i > 0) ss << "; ";
      for (int j = 0; j < matrix.cols(); ++j) {
        if (j > 0) ss << " ";
        ss << matrix(i,j);
      }
    }
  }
  return ss.str();
}

template<typename Derived>
std::string EncodeJson(const Eigen::DenseBase<Derived>& matrix) {
  std::string s = "[";
  if (matrix.cols() == 1) { // Column vector
    // [[1],[2],[3],[4]] => "[1,2,3,4]"
    for (int i = 0; i < matrix.rows(); ++i) {
      if (i > 0) s.append(",");
      s.append(std::to_string(matrix(i,0)));
    }
  } else { // Matrix
    // [[1,2,3,4]]   => "[1,2,3,4]"
    // [[1,2],[3,4]] => "[[1,2],[3,4]]"
    for (int i = 0; i < matrix.rows(); ++i) {
      if (i > 0) s.append(",");
      // Nest arrays only if there are multiple rows
      if (matrix.rows() > 1) s.append("[");
      for (int j = 0; j < matrix.cols(); ++j) {
        if (j > 0) s.append(",");
        s.append(std::to_string(matrix(i,j)));
      }
      // Nest arrays only if there are multiple rows
      if (matrix.rows() > 1) s.append("]");
    }
  }
  s.append("]");
  return s;
}

template<typename Derived>
Derived DecodeJson(const std::string& str) {
  // Find last nested row delimiter
  size_t idx_row_end = str.find_last_of(']');
  if (idx_row_end != std::string::npos) {
    size_t idx_temp = str.substr(0, idx_row_end).find_last_of(']');
    if (idx_temp != std::string::npos) idx_row_end = idx_temp;
  }

  // Count number of columns
  size_t num_cols = 0;
  size_t idx = 0;
  size_t idx_col_end = str.find_first_of(']');
  while (idx < idx_col_end) {
    // Skip over extra whitespace
    idx = str.find_first_not_of(' ', idx);
    if (idx >= idx_col_end) break;

    // Find next delimiter
    idx = str.find_first_of(',', idx + 1);
    ++num_cols;
  }
  if (idx > idx_col_end) idx = idx_col_end;

  // Count number of rows
  size_t num_rows = 1;  // First row already traversed
  while (idx < idx_row_end) {
    // Skip over irrelevant characters
    idx = str.find_first_not_of(']', idx);
    if (idx >= idx_row_end) break;

    // Find next delimiter
    idx = str.find_first_of(']', idx + 1);
    ++num_rows;
  }

  // Check number of rows and columns
  if (num_cols == 0)
    throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
  if (num_rows == 1) {
    // Convert to vector
    num_rows = num_cols;
    num_cols = 1;
  }

  // Parse matrix
  Eigen::MatrixXd matrix(num_rows, num_cols);
  std::string str_local(str);
  for (char delimiter : ",[]") {
    std::replace(str_local.begin(), str_local.end(), delimiter, ' ');
  }
  std::stringstream ss(str_local);
  for (size_t i = 0; i < num_rows; ++i) {
    for (size_t j = 0; j < num_cols; ++j) {
      std::string val;
      ss >> val;
      try {
        matrix(i,j) = std::stod(val);
      } catch (const std::exception& e) {
        throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
      }
    }
  }

  return matrix;
}

}  // namespace EigenUtils

#endif  // EIGEN_UTILS_H_
