#include "bayer_downscale_cuda.h"

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>

namespace
{
constexpr int kScale = 8;

uint16_t* d_bayer = nullptr;
uint8_t* d_bgr = nullptr;
size_t d_bayer_bytes = 0;
size_t d_bgr_bytes = 0;

std::string cuda_error_string(const char* operation, cudaError_t error)
{
  std::ostringstream ss;
  ss << operation << " failed: " << cudaGetErrorString(error);
  return ss.str();
}

cudaError_t ensure_device_buffer(void** ptr, size_t* current_bytes, size_t required_bytes)
{
  if (*current_bytes >= required_bytes) {
    return cudaSuccess;
  }

  if (*ptr) {
    cudaFree(*ptr);
    *ptr = nullptr;
    *current_bytes = 0;
  }

  const cudaError_t error = cudaMalloc(ptr, required_bytes);
  if (error != cudaSuccess) {
    return error;
  }

  *current_bytes = required_bytes;
  return cudaSuccess;
}

__device__ uint8_t average_to_u8(uint32_t sum, uint32_t count, int raw_shift)
{
  if (count == 0) {
    return 0;
  }

  uint32_t value = sum / count;
  if (raw_shift > 0) {
    value >>= raw_shift;
  } else if (raw_shift < 0) {
    value <<= -raw_shift;
  }
  return static_cast<uint8_t>(value > 255u ? 255u : value);
}

__global__ void bayer_rggb16_downscale8_to_bgr8_kernel(
    const uint16_t* __restrict__ bayer,
    int input_width,
    int input_height,
    uint8_t* __restrict__ bgr,
    int output_width,
    int output_height,
    int raw_shift)
{
  const int ox = blockIdx.x * blockDim.x + threadIdx.x;
  const int oy = blockIdx.y * blockDim.y + threadIdx.y;
  if (ox >= output_width || oy >= output_height) {
    return;
  }

  const int sx0 = ox * kScale;
  const int sy0 = oy * kScale;
  uint32_t r_sum = 0;
  uint32_t g_sum = 0;
  uint32_t b_sum = 0;
  uint32_t r_count = 0;
  uint32_t g_count = 0;
  uint32_t b_count = 0;

  for (int dy = 0; dy < kScale; ++dy) {
    const int sy = sy0 + dy;
    if (sy >= input_height) {
      continue;
    }

    const bool even_row = (sy & 1) == 0;
    const uint16_t* row = bayer + sy * input_width;
    for (int dx = 0; dx < kScale; ++dx) {
      const int sx = sx0 + dx;
      if (sx >= input_width) {
        continue;
      }

      const uint32_t value = row[sx];
      const bool even_col = (sx & 1) == 0;
      if (even_row && even_col) {
        r_sum += value;
        ++r_count;
      } else if (!even_row && !even_col) {
        b_sum += value;
        ++b_count;
      } else {
        g_sum += value;
        ++g_count;
      }
    }
  }

  const int out_idx = (oy * output_width + ox) * 3;
  bgr[out_idx + 0] = average_to_u8(b_sum, b_count, raw_shift);
  bgr[out_idx + 1] = average_to_u8(g_sum, g_count, raw_shift);
  bgr[out_idx + 2] = average_to_u8(r_sum, r_count, raw_shift);
}

}  // namespace

bool bayer_rggb16_downscale8_to_bgr8_cuda(
    const uint16_t* host_bayer,
    int input_width,
    int input_height,
    uint8_t* host_bgr,
    int output_width,
    int output_height,
    int raw_shift,
    std::string* error_message)
{
  auto set_error = [error_message](const std::string& msg) {
    if (error_message) {
      *error_message = msg;
    }
  };

  if (!host_bayer || !host_bgr) {
    set_error("null host input or output pointer");
    return false;
  }
  if (input_width <= 0 || input_height <= 0 || output_width <= 0 || output_height <= 0) {
    set_error("invalid input or output dimensions");
    return false;
  }

  const size_t bayer_bytes =
      static_cast<size_t>(input_width) * static_cast<size_t>(input_height) * sizeof(uint16_t);
  const size_t bgr_bytes =
      static_cast<size_t>(output_width) * static_cast<size_t>(output_height) * 3U;

  cudaError_t error = ensure_device_buffer(
      reinterpret_cast<void**>(&d_bayer), &d_bayer_bytes, bayer_bytes);
  if (error != cudaSuccess) {
    set_error(cuda_error_string("cudaMalloc input", error));
    return false;
  }
  error = ensure_device_buffer(
      reinterpret_cast<void**>(&d_bgr), &d_bgr_bytes, bgr_bytes);
  if (error != cudaSuccess) {
    set_error(cuda_error_string("cudaMalloc output", error));
    return false;
  }

  error = cudaMemcpy(d_bayer, host_bayer, bayer_bytes, cudaMemcpyHostToDevice);
  if (error != cudaSuccess) {
    set_error(cuda_error_string("cudaMemcpy input", error));
    return false;
  }

  const dim3 block(16, 16);
  const dim3 grid(
      (output_width + block.x - 1) / block.x,
      (output_height + block.y - 1) / block.y);

  bayer_rggb16_downscale8_to_bgr8_kernel<<<grid, block>>>(
      d_bayer,
      input_width,
      input_height,
      d_bgr,
      output_width,
      output_height,
      raw_shift);

  error = cudaGetLastError();
  if (error != cudaSuccess) {
    set_error(cuda_error_string("CUDA kernel launch", error));
    return false;
  }

  error = cudaDeviceSynchronize();
  if (error != cudaSuccess) {
    set_error(cuda_error_string("CUDA kernel sync", error));
    return false;
  }

  error = cudaMemcpy(host_bgr, d_bgr, bgr_bytes, cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) {
    set_error(cuda_error_string("cudaMemcpy output", error));
    return false;
  }

  return true;
}
