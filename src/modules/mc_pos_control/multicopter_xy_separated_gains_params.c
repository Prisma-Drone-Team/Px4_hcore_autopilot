
// *** CUSTOM ***

/**
 * Enable separated xy gains
 *
 * If enabled, position and velocity xy gains are separated. If disabled
 *
 * @boolean
 * @reboot_required true
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_XY_DIFFGAINS, 0);


/**
 * Proportional gain for horizontal position X error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_X_P, 0.95f);

/**
 * Proportional gain for horizontal position Y error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Y_P, 0.95f);



/**
 * Proportional gain for horizontal velocity X error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_X_VEL_P_ACC, 1.8f);

/**
 * Proportional gain for horizontal velocity Y error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Y_VEL_P_ACC, 1.8f);


/**
 * Integral gain for horizontal velocity X error
 *
 * Defined as correction acceleration in m/s^2 per m velocity integral
 * Allows to eliminate steady state errors in disturbances like wind.
 *
 * @min 0
 * @max 60
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_X_VEL_I_ACC, 0.4f);

/**
 * Integral gain for horizontal velocity Y error
 *
 * Defined as correction acceleration in m/s^2 per m velocity integral
 * Allows to eliminate steady state errors in disturbances like wind.
 *
 * @min 0
 * @max 60
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Y_VEL_I_ACC, 0.4f);


/**
 * Differential gain for horizontal velocity X error
 *
 * Defined as corrective acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.1
 * @max 2
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_X_VEL_D_ACC, 0.2f);

/**
 * Differential gain for horizontal Y velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.1
 * @max 2
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Y_VEL_D_ACC, 0.2f);


/**
 * Multiplier gain for servo pitch actuation
 *
 * multiply pitch setpoint from pos-vel controller before apply to servo
 *
 * @min 0.1
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_SERVO_K, 1.0f);
