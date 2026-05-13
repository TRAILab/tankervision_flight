#pragma once

#include <cstdint>
#include <string>

bool bayer_rggb16_downscale_to_bgr8_cuda(
    const uint16_t* host_bayer,
    int input_width,
    int input_height,
    uint8_t* host_bgr,
    int output_width,
    int output_height,
    int scale,
    int raw_shift,
    std::string* error_message);
