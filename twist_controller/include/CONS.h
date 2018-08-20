//
// Created by adam on 18-7-26.
//

#ifndef CONTROL_CONST_H
#define CONTROL_CONST_H

// controller param for throttle
#define THROTTLE_KP 0.27
#define THROTTLE_KI 0.3
#define THROTTLE_KD 0.0

#define THROTTLE_MIN 0.0
#define THROTTLE_MAX 1.0

// controller param for steering when speed is 0-10
#define STEER_0_10_KP 0.27
#define STEER_0_10_KI 0.0
#define STEER_0_10_KD 1.0


#define STEER_MIN_10 -20.0
#define STEER_MAX_10 20.0

// controller param for steering when speed is 10-20
#define STEER_10_20_KP 0.27
#define STEER_10_20_KI 0.0
#define STEER_10_20_KD 0.0

#define STEER_MIN_20 -10.0
#define STEER_MAX_20 10.0

// controller param for steering when speed is 20-30
#define STEER_20_30_KP 0.27
#define STEER_20_30_KI 0.0
#define STEER_20_30_KD 0.0

#define STEER_MIN_30 -7.0
#define STEER_MAX_30 7.0

// controller param for steering when speed is 30-40
#define STEER_30_40_KP 0.27
#define STEER_30_40_KI 0.0
#define STEER_30_40_KD 0.0

#define STEER_MIN_40 -5.0
#define STEER_MAX_40 5.0





#endif //CONTROL_CONST_H
