#ifndef LOCAL_TAIL_POS_TASK
#define LOCAL_TAIL_POS_TASK

#include "../../WBC/Task.hpp"

template <typename T> class FloatingBaseModel;

template <typename T>
class LocalTailPosTask : public Task<T> {
 public:
  LocalTailPosTask(const FloatingBaseModel<T>*);
  virtual ~LocalTailPosTask();

  DVec<T> _Kp_kin;
  DVec<T> _Kp, _Kd;

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                              const DVec<T>& acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }

  int link_idx_;
  bool virtual_depend_;
  const FloatingBaseModel<T>* _robot_sys;
};

#endif
