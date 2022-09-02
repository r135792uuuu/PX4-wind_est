
/**
 * Complimentary filter accelerometer weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_ACC1, 0.2f);

/**
 * Complimentary filter magnetometer weight
 *
 * Set to 0 to avoid using the magnetometer.
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_MAG1, 0.1f);

/**
 * Complimentary filter external heading weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(ATT_W_EXT_HDG1, 0.1f);

/**
 * Complimentary filter gyroscope bias weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_GYRO_BIAS1, 0.1f);

/**
 * Magnetic declination, in degrees
 *
 * This parameter is not used in normal operation,
 * as the declination is looked up based on the
 * GPS coordinates of the vehicle.
 *
 * @group Attitude Q estimator
 * @unit deg
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_MAG_DECL1, 0.0f);

/**
 * Automatic GPS based declination compensation
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_MAG_DECL_A1, 1);

/**
 * Acceleration compensation based on GPS
 * velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 PARAM_DEFINE_INT32(ATT_ACC_COMP1, 1);
 */


/**
 * External heading usage mode (from Motion capture/Vision)
 * Set to 1 to use heading estimate from vision.
 * Set to 2 to use heading from motion capture.
 *
 * @group Attitude Q estimator
 * @value 0 None
 * @value 1 Vision
 * @value 2 Motion Capture
 * @min 0
 * @max 2
  PARAM_DEFINE_INT32(ATT_EXT_HDG_M1, 0);
 */

/**
 * Gyro bias limit
 *
 * @group Attitude Q estimator
 * @unit rad/s
 * @min 0
 * @max 2
 * @decimal 3
  PARAM_DEFINE_FLOAT(ATT_BIAS_MAX1, 0.05f);

 */

