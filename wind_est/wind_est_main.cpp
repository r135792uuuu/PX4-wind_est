#include <cfloat>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

extern "C" __EXPORT int wind_est_main(int argc, char *argv[]);

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;
using matrix::Matrix;
//px4 don't have the type of Vector4f, 有空自己写一个

//the method to calculate motor speed
//also can write in parameter.c file to change in QGC
#define MotorRpmMethod 3
#define DCMCalculateMethod 1

class WindEst;

namespace wind_est
{
WindEst *instance;
} // namespace wind_est  为这个函数创建一个实例化对象，就算下面的class，包括什么构造函数析构函数 成员等


class WindEst
{
public:
	/**
         * Constructor 完成对私有变量的初始化（全部赋值0）
	 */
        WindEst();

	/**
	 * Destructor, also kills task.
	 */
        ~WindEst();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
        int		start();  //创建任务进程，使用px4自带的任务创建指令，然后跳转task_main_trampoline函数执行

        static int	task_main_trampoline(int argc, char *argv[]); //封装好的进程的执行函数，看下面解析，会跑到taskmain里进行执行，相当于封装的壳子

	void		task_main(); // 实际的姿态解散的程序入口，封装的内核


private:
	const float _dt_min = 0.00001f;   //微分时间最大最小值
	const float _dt_max = 0.02f;
        //飞机的各个常数变量，其实也可也放到参数服务器中，改成普遍型的 TODO
        //iris的属性
        float _m = 1.535;
        float _c_force = 5;
        float _c_torque = 0.5;
        float _g0 = -9.806;  //默认，后面精准计算

        float _Ct = 0.000175;
        float _Cm = 1e-06; //按照定义应该是 pitchingmomentcoeff，但是sdf文件只给了rollingmomentcoeff
        float _J11 = 0.029125;
        float _J22 = 0.029125;
        float _J33 = 0.055225;
        float _d = 0.247;

        //句柄变量设置以及进程任务变量设置
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

	int		_params_sub = -1;     //各种订阅者的名字 这个是参数更新器
        int		_sensor_combined_sub = -1;    //snesor_combined
	int		_global_pos_sub = -1;   //gps
	int		_magnetometer_sub = -1;    //电子罗盘
        int             _vehicle_acceleration_sub = -1;
        int             _vehicle_attitude_sub = -1;
        int             _vehicle_angular_velocity_sub = -1;
        int             _vehicle_angular_acceleration_sub = -1;
        int             _actuator_outputs_sub = -1;
        int             _vehicle_local_position_sub = -1;


        orb_advert_t	_vehicle_thrust_setpoint_pub = nullptr;
        orb_advert_t	_vehicle_torque_setpoint_pub = nullptr;
        //拷贝系统的默认参数

        //
	struct {
		param_t	w_acc;
		param_t	w_mag;
		param_t	w_ext_hdg;
		param_t	w_gyro_bias;
		param_t	mag_decl;
		param_t	mag_decl_auto;
                //param_t	ext_hdg_mode;
                //param_t	bias_max;
                //param_t	acc_comp;
		param_t	has_mag;
	} _params_handles{};		/**< handles for interesting parameters 提前定义参数服务器里的名字，在下面把参数服务器里的参数拷贝到当前进程中*/

        float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
        //bool		_acc_comp = false;
        //float		_bias_max = 0.0f;
        //int32_t		_ext_hdg_mode = 0;  //看下面，这里0代表选择电子露盘来补偿磁偏角

        //装载各种数据的容器在这里
	Vector3f	_gyro;
	Vector3f	_accel;
	Vector3f	_mag;

        Quatf		_q;     //飞机的初始姿态  px4: q0-q1 对应 w x y z
        Vector3f	_rates; //角速率
        Vector3f	_gyro_bias;   //预留出ekf，看是否需要ekf来精准的算这些，其实完全可以通过uorb得到

	Vector3f	_vel_prev;
	hrt_abstime	_vel_prev_t = 0;

        Vector3f        _vei_acc;   //直接从加速度消息读出来的，理论上讲应该是和我用IMU估计出来的一样 TODO融合互补
        Vector3f	_pos_acc;   //运动加速度，需要的
        Vector3f        _ang_acc;   //角加速度
        Vector3f        _ang_vel;   //角速度
        Vector3f        _ang_eul;     //欧拉角，弧度制
        Vector3f        _thrust;     //thrust
        Vector3f        _torque;        //torque
        Vector3f        _Ri3;       //DCM third column TODO: Dcmf R

        float_t        _output[4];    //mixer to pwm, from "pwm out sim"
        float_t        _Omega_Wn[4];  //motor rpm,aka motor speed

	bool		_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

	void update_parameters(bool force);    //参数更新，在把参数初始的值拷贝近来之后，需要更新参数，用这个函数

        //int update_subscriptions();  //TODO 提高代码的封装性

        bool init();

        bool update(float dt); //405行？姿态数据的更新函数 需要微分时间

	// Update magnetic declination (in rads) immediately changing yaw rotation
        void update_mag_declination(float new_declination);

        bool calculate_motor_speed(int method); //计算电机转速
};

/**
 *初始化工作，把初始的参数拷贝到进程中使用，参数的变量名在上面定义的    以及对容器0初始化
 */
WindEst::WindEst()
{       //类中各种变量的初始化 以及默认参数的赋值
        _params_handles.w_acc		= param_find("ATT_W_ACC1");
        _params_handles.w_mag		= param_find("ATT_W_MAG1");
        _params_handles.w_ext_hdg	= param_find("ATT_W_EXT_HDG1");
        _params_handles.w_gyro_bias	= param_find("ATT_W_GYRO_BIAS1");
        _params_handles.mag_decl	= param_find("ATT_MAG_DECL1");
        _params_handles.mag_decl_auto	= param_find("ATT_MAG_DECL_A1");
        //_params_handles.acc_comp	= param_find("ATT_ACC_COMP1");
        //_params_handles.bias_max	= param_find("ATT_BIAS_MAX1");
        //_params_handles.ext_hdg_mode	= param_find("ATT_EXT_HDG_M1");
	_params_handles.has_mag		= param_find("SYS_HAS_MAG");

	_vel_prev.zero();
	_pos_acc.zero();

	_gyro.zero();
	_accel.zero();
	_mag.zero();

	_q.zero();
	_rates.zero();
        _gyro_bias.zero();
        _ang_vel.zero();
        _Ri3.zero();
        _ang_eul.zero();
        _thrust.zero();
        _torque.zero();
        for(int i = 0; i < 4; i++)
        {
            _output[i] = 0.f;
            _Omega_Wn[i] = 0.f;
        }

}

/**
 * Destructor, also kills task.
 */
WindEst::~WindEst()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

        wind_est::instance = nullptr;
}

int WindEst::start()
{
        //ASSERT(_control_task == -1);

	/* start the task 创建进程*/
        //参数意义： 名字，进程默认调度，进程调度优先级，进程私有栈大小，进程的执行函数（下面有定义），参数列表（没有就算nullptr）
        _control_task = px4_task_spawn_cmd("wind_est",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ESTIMATOR,
					   2000,
                                           (px4_main_t)&WindEst::task_main_trampoline,
                                           nullptr); //跳转到进程的执行函数task_main_trampoline里面去，看下面

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int WindEst::task_main_trampoline(int argc, char *argv[])
{
        wind_est::instance->task_main();   //跳转到真正封装的姿态解算核心函数黎曼 就是下面这个task_main
	return 0;
}

void WindEst::task_main()
{  //真正的执行姿态结算进程的函数，封装的内核就算这个；
    //在这个进程函数里面调用姿态结算更新的核心函数，但是这个函数会被作为被px4注册的入口
//应该是类似考虑的重复定义的问题
#ifdef __PX4_POSIX
	perf_counter_t _perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay"));
	perf_counter_t _perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay"));
	perf_counter_t _perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay"));
#endif
        //订阅各类消息 他们的订阅句柄在上面类中定义了   
        _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
        _vehicle_acceleration_sub = orb_subscribe(ORB_ID(vehicle_acceleration));
        _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        _vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
        _vehicle_angular_acceleration_sub = orb_subscribe(ORB_ID(vehicle_angular_acceleration));
        _actuator_outputs_sub  = orb_subscribe(ORB_ID(actuator_outputs));
        _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

        update_parameters(true); //参数更新函数，这个是强制执行的。当传入false时是判断是否有变动，如有变动才会更新拷贝

	hrt_abstime last_time = 0;

        px4_pollfd_struct_t fds[1] = {}; //阻塞等待sensor_combined
        fds[0].fd = _actuator_outputs_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
                int ret = px4_poll(fds, 1, 1000);  //1000ms
		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			PX4_WARN("POLL ERROR");
			continue;  //不拿到数据就一直执行

		} else if (ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("POLL TIMEOUT");
			continue;  //不拿到数据就一直执行
		}
                update_parameters(false);   //拿到数据之后进行检查
                 //outputs from mixer to motor rpm(is not a specific equation, TODO)
                actuator_outputs_s s_output;
                if ( orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &s_output) == PX4_OK) {
                                                _output[0] = s_output.output[0];
                                                _output[1] = s_output.output[1];
                                                _output[2] = s_output.output[2];
                                                _output[3] = s_output.output[2];

                                }
			_data_good = true;


		bool magnetometer_updated = false;
		orb_check(_magnetometer_sub, &magnetometer_updated);
		if (magnetometer_updated) {
			vehicle_magnetometer_s magnetometer;

			if (orb_copy(ORB_ID(vehicle_magnetometer), _magnetometer_sub, &magnetometer) == PX4_OK) {
				_mag(0) = magnetometer.magnetometer_ga[0];
				_mag(1) = magnetometer.magnetometer_ga[1];
				_mag(2) = magnetometer.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_ERR("degenerate mag!");
					continue;
				}
			}

		}


                // Update attitude,其实应该和output一起阻塞等待。TODO
                bool vehicle_attitude_updated = false;
                orb_check(_vehicle_attitude_sub, &vehicle_attitude_updated);
                if (vehicle_attitude_updated) {
                        struct vehicle_attitude_s vehicle_attitude;

                        if (orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude) == PX4_OK) {
                                _q(0) = vehicle_attitude.q[0];
                                _q(1) = vehicle_attitude.q[1];
                                _q(2) = vehicle_attitude.q[2];
                                _q(3) = vehicle_attitude.q[3];
                                _q.copyTo(vehicle_attitude.delta_q_reset); //防止错误出现

                                }
                        else {
                            //int _quat_error = quat_reset_counter++;
                            _q(0) = vehicle_attitude.delta_q_reset[0];
                            _q(1) = vehicle_attitude.delta_q_reset[1];
                            _q(2) = vehicle_attitude.delta_q_reset[2];
                            _q(3) = vehicle_attitude.delta_q_reset[3];
                        }
                        }

                //拿到accel之后要和运动加速度减去才是重力加速度
                bool vehicle_acceleration_updated = false;
                orb_check(_vehicle_acceleration_sub, &vehicle_acceleration_updated);
                //有更新再拷贝出来，没有就算了,可以用IMU计算的来替代它
                if (vehicle_acceleration_updated) {
                        vehicle_acceleration_s vehicle_acceleration;

                                if ( orb_copy(ORB_ID(vehicle_acceleration), _vehicle_acceleration_sub, &vehicle_acceleration) == PX4_OK) {
                                        _vei_acc(0) = vehicle_acceleration.xyz[0];
                                        _vei_acc(1) = vehicle_acceleration.xyz[1];
                                        _vei_acc(2) = vehicle_acceleration.xyz[2];


                                        if (_vei_acc.length() < 0.01f) {
                                                PX4_ERR("degenerate veihcle acc!");
                                                continue;
                                        }

			}
		}

                bool vehicle_angular_velocity_updated = false;
                orb_check(_vehicle_angular_velocity_sub, &vehicle_angular_velocity_updated);
                if (vehicle_angular_velocity_updated) {
                        vehicle_angular_velocity_s vehicle_angular_velocity;

                                if ( orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_angular_velocity_sub, &vehicle_angular_velocity) == PX4_OK) {
                                        _ang_vel(0) = vehicle_angular_velocity.xyz[0];
                                        _ang_vel(1) = vehicle_angular_velocity.xyz[1];
                                        _ang_vel(2) = vehicle_angular_velocity.xyz[2];
                                    }
                                else {
                                    PX4_ERR("degenerate _ang_vel!");
                                    continue;
                                }
                }


                // Update sensors
                bool  sensor_combined_updated = false;
                orb_check(_sensor_combined_sub, & sensor_combined_updated);
                if(sensor_combined_updated){
                    sensor_combined_s sensors; //数据拷贝出来，到sensors容器里面
                   if (orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensors) == PX4_OK) {

                           // Feed validator with recent sensor data
                           //装载到我们预先在类中定义好的各种容器里面
                           if (sensors.timestamp > 0) {
                                   _gyro(0) = sensors.gyro_rad[0];
                                   _gyro(1) = sensors.gyro_rad[1];
                                   _gyro(2) = sensors.gyro_rad[2];
                           }
                           //！！！；这里是如何使用sensor combined里面的时间辍的用法
                           if (sensors.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
                                   _accel(0) = sensors.accelerometer_m_s2[0];
                                   _accel(1) = sensors.accelerometer_m_s2[1];
                                   _accel(2) = sensors.accelerometer_m_s2[2];

                                   if (_accel.length() < 0.01f) {
                                           PX4_ERR("degenerate accel!");
                                           continue;
                                   }
                           }
                   }
                }

                bool vehicle_angular_acceleration_updated = false;
                orb_check(_vehicle_angular_acceleration_sub, &vehicle_angular_acceleration_updated);
                if (vehicle_angular_acceleration_updated) {
                        vehicle_angular_acceleration_s vehicle_angular_acceleration;

                                if ( orb_copy(ORB_ID(vehicle_angular_acceleration), _vehicle_angular_acceleration_sub, &vehicle_angular_acceleration) == PX4_OK) {
                                        _ang_acc(0) = vehicle_angular_acceleration.xyz[0];
                                        _ang_acc(1) = vehicle_angular_acceleration.xyz[1];
                                        _ang_acc(2) = vehicle_angular_acceleration.xyz[2];
                                }

                                        else {
                                            PX4_ERR("degenerate _ang_acc!!");
                                            continue;
                                        }
                }


                bool vehicle_local_position_updated = false; //判断gps数据
                orb_check(_vehicle_local_position_sub, &vehicle_local_position_updated);
                if (vehicle_local_position_updated) {
                        vehicle_local_position_s vehicle_local_position;

                        if (orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &vehicle_local_position) == PX4_OK) {

                                if (hrt_elapsed_time(&vehicle_local_position.timestamp) < 20
                                    && vehicle_local_position.v_xy_valid && vehicle_local_position.v_z_valid && (vehicle_local_position.eph < 5.0f) && _inited) {

                                        /* position data is actual */
                                        const Vector3f vel(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);

                                        /* velocity updated */
                                        if (_vel_prev_t != 0 && vehicle_local_position.timestamp != _vel_prev_t) {
                                                float vel_dt = (vehicle_local_position.timestamp - _vel_prev_t) / 1e6f;
                                                //所需要的运动加速度
                                                _pos_acc = _q.conjugate_inversed((vel - _vel_prev) / vel_dt);
                                        }

                                        _vel_prev_t = vehicle_local_position.timestamp;
                                        _vel_prev = vel;

                                } else {
                                        //所需要的运动加速度
                                        _pos_acc.zero();
                                        _vel_prev.zero();
                                        _vel_prev_t = 0;
                                }
                        }
                }


                //计算重力加速度
                //_g0 =  _accel(2) - _pos_acc(2);
                //Test
                //PX4_INFO("_vei_acc:%f, _pos_acc:%f",_vei_acc(2),_pos_acc(2));


		/* time from previous iteration */
                hrt_abstime now = hrt_absolute_time();  //高精度计时器 获取绝对时间
		const float dt = math::constrain((now  - last_time) / 1e6f, _dt_min, _dt_max);
		last_time = now;

                if (update(dt)) {
                        vehicle_thrust_setpoint_s thrust = {};
                        _vehicle_thrust_setpoint_pub = orb_advertise(ORB_ID(vehicle_thrust_setpoint), &thrust);
                        thrust.timestamp = now;
                        //thrust.timestamp_sample = sensors.timestamp;
                        thrust.xyz[0] = _thrust(0);
                        thrust.xyz[1] = _thrust(1);
                        thrust.xyz[2] = _thrust(2);
                        orb_publish(ORB_ID( vehicle_thrust_setpoint), &_vehicle_thrust_setpoint_pub, &thrust);

                        vehicle_torque_setpoint_s torque = {};
                        _vehicle_torque_setpoint_pub = orb_advertise(ORB_ID(vehicle_torque_setpoint), &torque);
                        //torque.timestamp_sample = sensors.timestamp;
                        torque.timestamp = now;
                        torque.xyz[0] = _torque(0);
                        torque.xyz[1] = _torque(1);
                        torque.xyz[2] = _torque(2);
                        orb_publish(ORB_ID(vehicle_torque_setpoint), &_vehicle_torque_setpoint_pub, &torque);

                        //PX4_INFO("thrust:%d, %d, %d",_thrust(0),_thrust(1),_thrust(2));
                        //PX4_INFO("torque:%d, %d, %d",_torque(0),_torque(1),_torque(2));
		}
	}

#ifdef __PX4_POSIX
	perf_end(_perf_accel);
	perf_end(_perf_mpu);
	perf_end(_perf_mag);
#endif
//取消订阅
	orb_unsubscribe(_params_sub);
        orb_unsubscribe(_sensor_combined_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_magnetometer_sub);
        orb_unsubscribe(_vehicle_angular_acceleration_sub);
        orb_unsubscribe(_vehicle_angular_velocity_sub);
        orb_unsubscribe(_vehicle_acceleration_sub);
        orb_unsubscribe(_vehicle_attitude_sub);

}

//检查有没有在QGC里面把参数修改了
void WindEst::update_parameters(bool force)
{
	bool updated = force;

	if (!updated) {
		orb_check(_params_sub, &updated);
	}

	if (updated) {
            //把默认的参数拷贝到私有参数中。使用备份来修改，防止手抖改一次参数把我默认的参数都给改了。
		parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

		param_get(_params_handles.w_acc, &_w_accel);
                param_get(_params_handles.w_mag, &_w_mag);

		// disable mag fusion if the system does not have a mag
		if (_params_handles.has_mag != PARAM_INVALID) {
			int32_t has_mag;

			if (param_get(_params_handles.has_mag, &has_mag) == 0 && has_mag == 0) {
				_w_mag = 0.f;
			}
		}

		if (_w_mag < FLT_EPSILON) { // if the weight is zero (=mag disabled), make sure the estimator initializes
			_mag(0) = 1.f;
			_mag(1) = 0.f;
			_mag(2) = 0.f;
		}

		param_get(_params_handles.w_ext_hdg, &_w_ext_hdg);
		param_get(_params_handles.w_gyro_bias, &_w_gyro_bias);

		float mag_decl_deg = 0.0f;
		param_get(_params_handles.mag_decl, &mag_decl_deg);
		update_mag_declination(math::radians(mag_decl_deg));

		int32_t mag_decl_auto_int;
		param_get(_params_handles.mag_decl_auto, &mag_decl_auto_int);
		_mag_decl_auto = (mag_decl_auto_int != 0);

                //int32_t acc_comp_int;
                //param_get(_params_handles.acc_comp, &acc_comp_int);
                //_acc_comp = (acc_comp_int != 0);

                //param_get(_params_handles.bias_max, &_bias_max);
                //param_get(_params_handles.ext_hdg_mode, &_ext_hdg_mode);
	}
}


//初始姿态的获取 就是第一步拿四元数初值 获取初始的q0 q1 q2 q3
bool WindEst::init()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
        // 'k' is Earth Z axis (Down) unit vector in body frame
        //用加速度代表z轴，下面用磁力计代表x轴，保证x z正交
        //第四步，从四元数里获取重力向量和磁场向量
        Vector3f k = -_accel;  //坐标系不一样，符号不一样
        k.normalize();  //归一化

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector3f i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
        Vector3f j = k % i;  //这是C++的叉乘，两个向量中间%，算出y轴

        // Fill rotation matrix  三个轴填充到旋转矩阵中
	Dcmf R;
	R.setRow(0, i);
	R.setRow(1, j);
	R.setRow(2, k);

        //好像新、老版本的DCM直接就是强制转换四元数=上面计算出来的旋转矩阵  上古版本的需要转换函数
	_q = R;

        //磁偏角补偿四元数得到符合地理的四元数
	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
	_q = _q * decl_rotation;

        _q.normalize();  //归一化四元数 我们就得到了四元数的4个初值

                //判断有没有成功
	if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	    PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
	    _q.length() > 0.95f && _q.length() < 1.05f) {
		_inited = true;

	} else {
		_inited = false;
	}

	return _inited;
}

bool WindEst::calculate_motor_speed(int method)
{
    if(method == 1)
    {//采用rpm的方式读取转速
        return true;

    }
    else if (method == 2) { //采用控制量乘以mixer矩阵（rpyt的分配矩阵）的转置来求转速
        //和hil方式差不多，乘出来也是PWM所以直接用hil方式
        return true;
    }
    else if (method == 3) { //采用hil方式来转速
        _Omega_Wn[0] = (_output[0] + 1.0f) / 2.0f;
        _Omega_Wn[1] = (_output[1] + 1.0f) / 2.0f;
        _Omega_Wn[2] = (_output[2] + 1.0f) / 2.0f;
        _Omega_Wn[3] = (_output[3] + 1.0f) / 2.0f;
        return  true;

    }
    else {
        //to add other methods
        PX4_INFO("Motor Reset");
        return  false;
    }

}

//姿态解算核心函数 实现四元数的更新 笔记上详细的步骤
bool WindEst::update(float dt)
{
	if (!_inited) {

		if (!_data_good) {
			return false;
		}

                return init(); //初始姿态的获取 就算第一步获取初始的q0 q1 q2 q3
	}

	Quatf q_last = _q;


        if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
              PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) {
                _q = q_last;
                _gyro_bias.zero();

        }

        // 获取姿态q  其实也可以用to_dcm函数直接得到，TODO 对比
        //TODO:??似乎新版本中可以直接运行器强制转换。例如 Dcmf R_DCM;R_DCM = _q;
        if(DCMCalculateMethod)
        {
            _Ri3(0) = 2.0f*(_q(1)*_q(3) - _q(0)*_q(2));
            _Ri3(1) = 2.0f*(_q(2)*_q(3) - _q(0)*_q(1));
            _Ri3(2) = (_q(0)*_q(0) - _q(1)*_q(1) - _q(2)*_q(2) + _q(3)*_q(3));       }
        else {

            Dcmf R;
            R = _q;
            Vector3f R_z;
            R_z(0) = R(0, 2);
            R_z(1) = R(1, 2);
            R_z(2) = R(2, 2);
        }

        //quat求姿态角  TODO:Eulerf eul_angle 暂时用不上
        Vector3f eul_angle(0,0,0);


        //求转速和控制输入
        calculate_motor_speed(MotorRpmMethod);


        //计算主要流程  公式4和公式13 TODO 目前是公式4的quad_x iris实现。要改变其他型号的飞机需要继续TODO if else
        //直接使用归一化作为结果参数运算。也就是转速限制在[0 1]上。因为后续发布的f_sp和torque_sp都还要通过mixer，所以需要这里归一化好
        float uf = 0;
        Vector3f utorque(0,0,0);
        //TODO 考虑有分配矩阵(力和力矩的分配矩阵)的形式给他乘出来，更concise
        uf = _Ct * (_Omega_Wn[0] * _Omega_Wn[0] + _Omega_Wn[1] * _Omega_Wn[1] +_Omega_Wn[2] * _Omega_Wn[2] +_Omega_Wn[3] * _Omega_Wn[3]);
        utorque(0) = _Ct * 0.707f * _d * (- _Omega_Wn[0] * _Omega_Wn[0] + _Omega_Wn[1] * _Omega_Wn[1] + _Omega_Wn[2] * _Omega_Wn[2] - _Omega_Wn[3] * _Omega_Wn[3]);
        utorque(1) = _Ct * 0.707f * _d * (_Omega_Wn[0] * _Omega_Wn[0] - _Omega_Wn[1] * _Omega_Wn[1] + _Omega_Wn[2] * _Omega_Wn[2] - _Omega_Wn[3] * _Omega_Wn[3]);
        utorque(2) = _Cm * (_Omega_Wn[0] * _Omega_Wn[0] + _Omega_Wn[1] * _Omega_Wn[1] - _Omega_Wn[2] * _Omega_Wn[2] - _Omega_Wn[3] * _Omega_Wn[3]);

        //calculate wrench
        float k = dt * _c_force / _m;
        //force xyz c-force coeff can be setted seperately
        //TODO accelration和posacc一阶融合
        _thrust(0) = (1-k)*_thrust(0) + k * (_m * _pos_acc(0) - _Ri3(0) * uf);
        _thrust(1) = (1-k)*_thrust(1) + k * (_m * _pos_acc(1) - _Ri3(1) * uf);
        _thrust(2) = (1-k)*_thrust(0) + k * (_m * _pos_acc(2) + _g0 - _Ri3(2) * uf);

        //torque: the analysis solution is from matlab
        _torque(0) = _ang_acc(0)*_c_torque*dt - _torque(0)*((dt*_c_torque)/_J11 - 1) - (dt*_c_torque*utorque(0))/_J11 - (_J22*dt*_ang_vel(1)*_c_torque*_ang_vel(0))/_J11 + (_J33*dt*_ang_vel(2)*_c_torque*_ang_vel(1))/_J22;
        _torque(1) = _ang_acc(1)*_c_torque*dt - _torque(1)*((dt*_c_torque)/_J22 - 1) - (dt*_c_torque*utorque(1))/_J22 + (_J11*dt*_ang_vel(0)*_c_torque*_ang_vel(2))/_J22 - (_J33*dt*_ang_vel(2)*_c_torque*_ang_vel(0))/_J22;
        _torque(2) = _ang_acc(2)*_c_torque*dt - _torque(2)*((dt*_c_torque)/_J33 - 1) - (dt*_c_torque*utorque(2))/_J33 - (_J11*dt*_ang_vel(0)*_c_torque*_ang_vel(1))/_J33 + (_J22*dt*_ang_vel(1)*_c_torque*_ang_vel(0))/_J33;


	return true;
}
//更新磁偏角的补偿 但是好像update里面好像直接包含了这段代码
void WindEst::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	if (!_inited || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
		_q = _q * decl_rotation;
		_mag_decl = new_declination;
	}
}
/**
extern "C" __EXPORT int wind_est_main(int argc, char *argv[])
{
        return WindEst::main(argc, argv);
}
*/
int wind_est_main(int argc, char *argv[])
{
    //主函数入口
	if (argc < 2) {
                warnx("usage: wind_est {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) { //对比输入参数是哪个

                if (wind_est::instance != nullptr) {
			warnx("already running");
			return 1;
		}

                wind_est::instance = new WindEst;

                if (wind_est::instance == nullptr) {
			warnx("alloc failed");
			return 1;
		}

                if (OK != wind_est::instance->start()) {
                        delete wind_est::instance;
                        wind_est::instance = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
                if (wind_est::instance == nullptr) {
			warnx("not running");
			return 1;
		}

                delete wind_est::instance;
                wind_est::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
                if (wind_est::instance) {
			warnx("running");
			return 0;
                        //PX4_INFO("thrust:%d, %d, %d",_thrust(0),_thrust(1),_thrust(2));
                        //PX4_INFO("torque:%d, %d, %d",_torque(0),_torque(1),_torque(2));
                        //PX4_INFO("_vei_acc:%f, _pos_acc:%f",_vei_acc(2),_pos_acc(2));

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

