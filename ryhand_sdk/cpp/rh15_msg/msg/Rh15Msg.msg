uint8 lr    # left_or_right

uint8[15]   status
float32[15] m_pos
float32[15] m_spd       
float32[15] m_cur  
float32[15] m_force 

# deg
float32[20] j_ang
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

float64[5] w
float64[5] i
float64[5] j 
float64[5] k