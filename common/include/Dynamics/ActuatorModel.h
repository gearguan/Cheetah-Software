/*! @file ActuatorModel.h
 *  @brief Model of actuator 电机建模
 *  Includes friction, max torque, and motor torque speed curve. 包括摩擦、最大扭矩和扭矩-转速曲线
 *
 *  The getTorque is used for torque at the joint, not torque at the motor.
 *  The provided frictions are for torques at the joint, not torque at the motor 摩擦是关节的，而非电机的
 *  The R/KT are for the motor
 */

#ifndef PROJECT_ACTUATORMODEL_H
#define PROJECT_ACTUATORMODEL_H

#include "Utilities/utilities.h"

/*!
 * A model of an actuator containing friction and electrical effects 包含摩擦和电气特性的驱动器模型
 */
template <typename T>
class ActuatorModel {
 public:

  /*!
   * Construct a new actuator model with the given parameters
   * @param gearRatio : Gear reduction 减速比
   * @param motorKT : Value of KT (torque constant) for the motor 电机扭矩常数(转矩系数) Kt
   * @param motorR : Motor resistance 电机电阻
   * @param batteryV : Battery voltage 电源电压
   * @param damping : Actuator damping (at the joint, Nm/(rad/sec)) 黏性阻尼系数
   // 在无刷直流电机中，黏性阻尼系数D 是一个重要参数。1.它表示了电机机械特性的硬度，D越大，机械特性硬度越硬，负载转矩单位增量引起的转速下降越小；
   // 2.D越大，电机响应越快；3.D越大，电磁效率越高。  
   * @param dryFriction : Actuator dry friction (at the joint, Nm)
   * @param tauMax : Maximum torque output of the actuator
   */
  ActuatorModel(T gearRatio, T motorKT, T motorR, T batteryV, T damping,
                T dryFriction, T tauMax)
      : _gr(gearRatio),
        _kt(motorKT),
        _R(motorR),
        _V(batteryV),
        _damping(damping),
        _dryFriction(dryFriction),
        _tauMax(tauMax) {}

  ActuatorModel() {}

  // compute

  /*!
   * Compute actual actuator torque, given desired torque and speed. 给定期望的扭矩和转速
   * takes into account friction (dry and damping), voltage limits, and torque
   * limits
   * @param tauDes : desired torque
   * @param qd : current actuator velocity (at the joint)
   * @return actual produced torque
   */
  T getTorque(T tauDes, T qd) {
    // compute motor torque
    T tauDesMotor = tauDes / _gr;        // motor torque
    T iDes = tauDesMotor / (_kt * 1.5);  // i = tau / KT 电枢电流 armature current？ 
    // T bemf =  qd * _gr * _kt * 1.732;     // back emf 反电动势（counter emf或back emf）是指有反抗电流通过趋势的电动势，其本质上属于感应电动势。
    T bemf = qd * _gr * _kt * 2.;       // back emf
    T vDes = iDes * _R + bemf;          // v = I*R + emf
    T vActual = coerce(vDes, -_V, _V);  // limit to battery voltage
    T tauActMotor =
        1.5 * _kt * (vActual - bemf) / _R;  // tau = Kt * I = Kt * V / R
    T tauAct = _gr * coerce(tauActMotor, -_tauMax, _tauMax);

    // add damping and dry friction
    if (_frictionEnabled)
      tauAct = tauAct - _damping * qd - _dryFriction * sgn(qd);

    return tauAct;
  }

  /*!
   * Control friction effects
   * @param enabled : enable/disable both dry and damping friction terms
   */
  void setFriction(bool enabled) { _frictionEnabled = enabled; }

 private:
  T _gr, _kt, _R, _V, _damping, _dryFriction, _tauMax;
  bool _frictionEnabled = true;
};

#endif  // PROJECT_ACTUATORMODEL_H
