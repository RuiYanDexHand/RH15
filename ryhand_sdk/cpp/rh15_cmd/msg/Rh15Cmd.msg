uint8 mode
uint8 lr       # left_or_right

# mode 0 - raw dat 
float32[15] m_pos
float32[15] m_spd       # mode 0，1，2
float32[15] m_curlimit  # mode 0，1，2

# mode 1  joint , deg
float32[20] j_ang

# mode 2  end pos
float64 x_base
float64 y_base
float64 z_base

float64 roll_base
float64 pitch_base
float64 yaw_base

float64[5] x
float64[5] y
float64[5] z

float64[5] roll
float64[5] pitch
float64[5] yaw